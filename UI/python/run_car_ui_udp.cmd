@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
 echo Run setup.cmd first.
 pause
 exit /b 1
)
if "%~1"=="" (
 echo Usage: run_car_ui.cmd ZYBO_COM_PORT [ESP32_CAM_STREAM_URL ^| OFF]
 echo Example: run_car_ui.cmd COM4
 pause
 exit /b 1
)
if /I "%~2"=="OFF" (
 ".venv\Scripts\python.exe" main.py --serial-port "%~1" --udp --host 0.0.0.0 --port 7000 --video-port 7001 --board-ip 192.168.10.2 --stats
) else (
 if "%~2"=="" (
  ".venv\Scripts\python.exe" main.py --serial-port "%~1" --udp --host 0.0.0.0 --port 7000 --video-port 7001 --board-ip 192.168.10.2 --esp32-cam-url auto --stats
 ) else (
  ".venv\Scripts\python.exe" main.py --serial-port "%~1" --udp --host 0.0.0.0 --port 7000 --video-port 7001 --board-ip 192.168.10.2 --esp32-cam-url "%~2" --stats
 )
)
pause
