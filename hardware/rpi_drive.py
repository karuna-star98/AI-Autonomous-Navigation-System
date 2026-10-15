"""OPTIONAL sim-to-real stub for a Raspberry Pi robot (NOT tested on real hardware).

The simulator's Navigator outputs (v, w). On a real robot those numbers must be
turned into left/right wheel commands and ranges must come from real sensors.
This file shows that thin hardware layer.

WIRING FIXES vs. the original project brief (please read hardware/README.md):
  * The brief assigns GPIO 23 to BOTH ultrasonic TRIG and motor IN4 -> pin conflict.
    Here IN4 uses GPIO 27.
  * HC-SR04 ECHO outputs 5 V; Raspberry Pi GPIO is 3.3 V only -> use a voltage divider.
"""
import time

try:
    import RPi.GPIO as GPIO          # only exists on a Raspberry Pi
except ImportError:                  # lets the file be imported/linted on a laptop
    GPIO = None

TRIG, ECHO = 23, 24
IN1, IN2, IN3, IN4 = 17, 18, 22, 27
ENA, ENB = 12, 13                    # hardware-PWM capable pins
WHEEL_BASE_M = 0.16
MAX_WHEEL_SPEED = 0.5                # m/s that corresponds to 100 % duty cycle


class PiRobot:
    def __init__(self):
        if GPIO is None:
            raise RuntimeError("RPi.GPIO not available - run this on a Raspberry Pi.")
        GPIO.setmode(GPIO.BCM)
        for p in (TRIG, IN1, IN2, IN3, IN4, ENA, ENB):
            GPIO.setup(p, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)
        self.pwm_a, self.pwm_b = GPIO.PWM(ENA, 1000), GPIO.PWM(ENB, 1000)
        self.pwm_a.start(0); self.pwm_b.start(0)

    def distance_cm(self, timeout=0.03):
        GPIO.output(TRIG, True); time.sleep(1e-5); GPIO.output(TRIG, False)
        t0 = time.time()
        while GPIO.input(ECHO) == 0:
            start = time.time()
            if start - t0 > timeout:
                return None
        while GPIO.input(ECHO) == 1:
            end = time.time()
            if end - t0 > timeout:
                return None
        return (end - start) * 17150.0

    def drive(self, v, w):
        """Unicycle (v m/s, w rad/s) -> differential drive wheel speeds."""
        left = v - w * WHEEL_BASE_M / 2
        right = v + w * WHEEL_BASE_M / 2
        for pwm, fwd, bwd, s in ((self.pwm_a, IN1, IN2, left), (self.pwm_b, IN3, IN4, right)):
            GPIO.output(fwd, s > 0); GPIO.output(bwd, s < 0)
            pwm.ChangeDutyCycle(min(100.0, abs(s) / MAX_WHEEL_SPEED * 100.0))

    def stop(self):
        self.drive(0, 0)

    def cleanup(self):
        self.stop(); GPIO.cleanup()


if __name__ == "__main__":
    bot = PiRobot()
    try:
        while True:
            d = bot.distance_cm()
            if d is not None and d < 25:
                bot.stop()
            else:
                bot.drive(0.2, 0.0)
            time.sleep(0.05)
    except KeyboardInterrupt:
        bot.cleanup()
