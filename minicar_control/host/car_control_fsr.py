import tkinter as tk
import time
import re
import serial

# ==================================================
# Zybo UART/PROG COM port
#
# Change this to the COM port shown for the Zybo.
# Example: "COM7"
# ==================================================
SERIAL_PORT = "COM14"
BAUD_RATE = 115200

SEND_PERIOD_MS = 20
STEERING_STEP_TIME = 0.10


# ==================================================
# Serial
#
# This replaces the old UDP path.
# PC -> UART1 -> Zybo
# ==================================================
ser = serial.Serial(
    SERIAL_PORT,
    BAUD_RATE,
    timeout=0
)


held_keys = set()
press_time = {}

emergency_stop = False
steering = 0

# Values reported back by the Zybo
accel_raw = 0
brake_raw = 0
accel_level = 0
brake_level = 0
pc_link = 0

serial_rx_buffer = ""


def key_name(event):
    return event.keysym.lower()


def get_hold_time(name):
    if name not in held_keys:
        return None

    return (
        time.monotonic()
        -
        press_time[name]
    )


def steering_value(hold_time):
    if hold_time is None:
        return 0

    step = 1 + int(
        hold_time / STEERING_STEP_TIME
    )

    angle = step * 10

    return min(angle, 90)


def on_key_press(event):
    global emergency_stop

    key = key_name(event)

    if key == "space":
        emergency_stop = True
        print("!!! EMERGENCY STOP !!!")
        return

    if key == "r":
        emergency_stop = False
        print("Emergency stop released")
        return

    if key == "q":
        safe_exit()
        return

    # Accel / Brake are no longer keyboard-controlled.
    # LEFT / RIGHT are the only direction keys used here.
    if key in {"left", "right"}:
        if key not in held_keys:
            held_keys.add(key)
            press_time[key] = time.monotonic()


def on_key_release(event):
    key = key_name(event)

    if key in held_keys:
        held_keys.remove(key)
        press_time.pop(
            key,
            None
        )


def update_steering():
    global steering

    if emergency_stop:
        steering = 0
        return

    left_time = get_hold_time("left")
    right_time = get_hold_time("right")

    if (
        left_time is not None
        and
        right_time is None
    ):
        steering = -steering_value(
            left_time
        )

    elif (
        right_time is not None
        and
        left_time is None
    ):
        steering = steering_value(
            right_time
        )

    else:
        steering = 0


def send_command():
    # Protocol:
    # K,<steering>,<estop>\n
    packet = (
        f"K,{steering},{1 if emergency_stop else 0}\n"
    )

    try:
        ser.write(
            packet.encode("ascii")
        )

    except Exception as e:
        print(
            "SERIAL TX ERROR:",
            e
        )


FSR_PATTERN = re.compile(
    r"FSR ACC_RAW=(-?\d+) A=(\d+) "
    r"BRAKE_RAW=(-?\d+) B=(\d+) "
    r"STEER=(-?\d+) ESTOP=(\d+) PC=(\d+)"
)


def handle_zybo_line(line):
    global accel_raw
    global brake_raw
    global accel_level
    global brake_level
    global pc_link

    match = FSR_PATTERN.search(line)

    if match:
        accel_raw = int(
            match.group(1)
        )

        accel_level = int(
            match.group(2)
        )

        brake_raw = int(
            match.group(3)
        )

        brake_level = int(
            match.group(4)
        )

        pc_link = int(
            match.group(7)
        )

    else:
        # Keep useful Vitis/Zybo messages visible.
        if line.strip():
            print(
                "[ZYBO]",
                line
            )


def read_zybo():
    global serial_rx_buffer

    try:
        waiting = ser.in_waiting

        if waiting > 0:
            data = ser.read(
                waiting
            ).decode(
                "ascii",
                errors="ignore"
            )

            serial_rx_buffer += data

            while "\n" in serial_rx_buffer:
                line, serial_rx_buffer = (
                    serial_rx_buffer.split(
                        "\n",
                        1
                    )
                )

                handle_zybo_line(
                    line.strip()
                )

    except Exception as e:
        print(
            "SERIAL RX ERROR:",
            e
        )


def update_display():
    steering_label.config(
        text=f"Steering : {steering:+d} deg"
    )

    accel_label.config(
        text=(
            f"Acceleration FSR : "
            f"RAW {accel_raw} / LEVEL {accel_level}"
        )
    )

    brake_label.config(
        text=(
            f"Brake FSR : "
            f"RAW {brake_raw} / LEVEL {brake_level}"
        )
    )

    if emergency_stop:
        status_label.config(
            text="EMERGENCY STOP"
        )
    else:
        status_label.config(
            text=(
                "DRIVING"
                if pc_link
                else
                "WAITING FOR ZYBO"
            )
        )


def control_loop():
    update_steering()

    send_command()

    read_zybo()

    update_display()

    root.after(
        SEND_PERIOD_MS,
        control_loop
    )


def safe_exit():
    print(
        "Stopping vehicle..."
    )

    # Send latched emergency stop repeatedly.
    for _ in range(10):
        try:
            ser.write(
                b"K,0,1\n"
            )
        except Exception:
            pass

        time.sleep(
            0.02
        )

    ser.close()

    root.destroy()


# ==================================================
# GUI
# ==================================================
root = tk.Tk()

root.title(
    "Mini Car - PC Steering + FSR Pedals"
)

root.geometry(
    "620x430"
)


title_label = tk.Label(
    root,
    text="MINI CAR HYBRID CONTROL",
    font=("Arial", 22, "bold")
)

title_label.pack(
    pady=20
)


steering_label = tk.Label(
    root,
    text="Steering : 0 deg",
    font=("Arial", 18)
)

steering_label.pack(
    pady=8
)


accel_label = tk.Label(
    root,
    text="Acceleration FSR : RAW 0 / LEVEL 0",
    font=("Arial", 16)
)

accel_label.pack(
    pady=8
)


brake_label = tk.Label(
    root,
    text="Brake FSR : RAW 0 / LEVEL 0",
    font=("Arial", 16)
)

brake_label.pack(
    pady=8
)


status_label = tk.Label(
    root,
    text="WAITING FOR ZYBO",
    font=("Arial", 18, "bold")
)

status_label.pack(
    pady=15
)


help_label = tk.Label(
    root,
    text=(
        "← / → : Steering\n"
        "FSR A0 : Accelerator\n"
        "FSR A1 : Brake\n"
        "SPACE : Emergency Stop\n"
        "R : Release Emergency Stop\n"
        "Q : Quit"
    ),
    font=("Arial", 13),
    justify="left"
)

help_label.pack(
    pady=10
)


root.bind(
    "<KeyPress>",
    on_key_press
)

root.bind(
    "<KeyRelease>",
    on_key_release
)

root.protocol(
    "WM_DELETE_WINDOW",
    safe_exit
)

root.focus_force()

root.after(
    SEND_PERIOD_MS,
    control_loop
)


print(
    "====================================="
)
print(
    " MINI CAR: PC STEERING + FSR PEDALS"
)
print(
    "====================================="
)
print(
    "LEFT/RIGHT = Steering"
)
print(
    "FSR A0     = Accelerator"
)
print(
    "FSR A1     = Brake"
)
print(
    "SPACE      = Emergency Stop"
)
print(
    "R          = Release Emergency Stop"
)
print(
    "Q          = Quit"
)
print()
print(
    f"Serial = {SERIAL_PORT} @ {BAUD_RATE}"
)


root.mainloop()
