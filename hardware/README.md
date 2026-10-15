# Optional: taking the project from simulation to a real robot

`rpi_drive.py` is a thin hardware layer (ultrasonic + L298N motor driver) showing how the
simulator's `(v, w)` commands map to wheel PWM. **It has not been tested on hardware** - treat it as a starting point.

## Two problems in the original wiring guide (fixed here)

| Issue | Original brief | Fix |
|-------|----------------|-----|
| GPIO conflict | Ultrasonic **TRIG = GPIO 23** *and* motor **IN4 = GPIO 23** | IN4 -> **GPIO 27** |
| Voltage level | HC-SR04 **ECHO (5 V)** wired straight to a Pi GPIO (**3.3 V max**) | Add a divider: ECHO -> 1 kΩ -> GPIO 24, and GPIO 24 -> 2 kΩ -> GND |
| Motor power | L298N powered from the Pi's 5 V pin | Power motors from the **battery** into the L298N 12 V terminal; share **GND** with the Pi |

Final pin map: TRIG 23, ECHO 24 (via divider), IN1 17, IN2 18, IN3 22, IN4 27, ENA 12 (PWM), ENB 13 (PWM).
