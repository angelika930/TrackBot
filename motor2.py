import gpiod
import time

# Define chip and line offsets (BCM pin numbers)
chip = gpiod.Chip("gpiochip4")  # Use `gpiodetect` to confirm chip name

# GPIO line numbers (BCM)
motor1_rpwm = 12
motor1_ren = 5
motor2_rpwm = 16
motor2_ren = 21

# Get lines from the chip
lines = chip.get_lines([motor1_rpwm, motor1_ren, motor2_rpwm, motor2_ren])

# Request lines as outputs with initial low (off)
lines.request(consumer="motor_forward", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0, 0, 0, 0])

# Turn on motors (simulate forward movement)
lines.set_values([1, 1, 1, 1])  # [motor1_rpwm, motor1_ren, motor2_rpwm, motor2_ren]

print("Motors running forward. Press Ctrl+C to stop.")

try:
    while True:
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping motors.")
    lines.set_values([0, 0, 0, 0])
    lines.release()

