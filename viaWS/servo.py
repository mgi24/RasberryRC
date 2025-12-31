import time
import pigpio

SERVO_PIN = 12  # BCM

pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpio daemon not running. Start with: sudo systemctl start pigpiod")

def set_angle(angle: float):
    angle = max(0.0, min(180.0, angle))
    # map 0..180 -> 500..2500 us (tweak if needed)
    pulse = 500 + (angle / 180.0) * (2500 - 500)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse)

try:
    set_angle(0)
    time.sleep(1)
    set_angle(180)
    time.sleep(1)
finally:
    pi.set_servo_pulsewidth(SERVO_PIN, 0)
    pi.stop()