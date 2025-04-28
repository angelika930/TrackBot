# Alternative using pigpio (better PWM performance)
# First install: sudo apt install pigpio python3-pigpio
# Then start daemon: sudo pigpiod

import pigpio
import time

# Initialize pigpio
pi = pigpio.pi()

# Same pin definitions as before
motor1_rpwm = 12
# ... other pin definitions remain the same

# Set pins as outputs
for pin in [motor1_rpwm, motor1_lpwm, motor1_ren, motor1_len,
           motor2_rpwm, motor2_lpwm, motor2_ren, motor2_len]:
    pi.set_mode(pin, pigpio.OUTPUT)

def forward(speed1=70, speed2=70):
    """
    Drive motors forward using pigpio
    speed range is 0-100 (converted to 0-255 PWM range)
    """
    # Convert 0-100 to 0-255 for pigpio
    pwm1 = int(speed1 * 2.55)
    pwm2 = int(speed2 * 2.55)
    
    # Enable forward direction
    pi.write(motor1_ren, 1)
    pi.write(motor1_len, 0)
    pi.write(motor2_ren, 1)
    pi.write(motor2_len, 0)
    
    # Set PWM values (0-255)
    pi.set_PWM_dutycycle(motor1_rpwm, pwm1)
    pi.set_PWM_dutycycle(motor1_lpwm, 0)
    pi.set_PWM_dutycycle(motor2_rpwm, pwm2)
    pi.set_PWM_dutycycle(motor2_lpwm, 0)
    
    print(f"Motors running forward: Motor 1 at {speed1}%, Motor 2 at {speed2}%")

# Similar implementations for stop() and other functions
# ...

# When done, clean up
# pi.stop()
