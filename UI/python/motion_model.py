"""Estimate ESP32 vehicleSpeed command state, not measured road speed."""
class MotionModel:
    ACCEL = (0, 20.4, 35.7, 56.1, 81.6, 114.8)
    BRAKE = (0, 25.5, 45.9, 76.5, 127.5, 204.0)
    COAST = 10.2
    def __init__(self):
        self.speed = 0.0
        self.reverse = False
    def step(self, accel_level, brake_level, dt, active=True, stopped=False, reverse=False):
        if not active or stopped:
            self.speed = 0.0
        else:
            a = max(0, min(5, int(accel_level)))
            b = max(0, min(5, int(brake_level)))
            rate = -self.BRAKE[b] if b else self.ACCEL[a] if a else -self.COAST
            self.speed = max(0.0, min(100.0, self.speed + rate * max(0.0, dt) * 100 / 255))
        # This supplied firmware ignores the reverse flag; do not reset state on it.
        self.reverse = reverse
        return self.speed
