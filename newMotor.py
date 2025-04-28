import RPi.GPIO as GPIO
import time

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)  # Use Broadcom pin-numbering scheme

# Motor 1 Pin Definitions
motor1_rpwm = 12  # GPIO pin connected to RPWM of motor controller 1
motor1_lpwm = 13  # GPIO pin connected to LPWM of motor controller 1
motor1_ren = 5    # GPIO pin connected to R_EN of motor controller 1
motor1_len = 6    # GPIO pin connected to L_EN of motor controller 1

# Motor 2 Pin Definitions
motor2_rpwm = 16  # GPIO pin connected to RPWM of motor controller 2
motor2_lpwm = 20  # GPIO pin connected to LPWM of motor controller 2
motor2_ren = 21   # GPIO pin connected to R_EN of motor controller 2
motor2_len = 26   # GPIO pin connected to L_EN of motor controller 2

# Set up all pins as outputs
for pin in [motor1_rpwm, motor1_lpwm, motor1_ren, motor1_len,
           motor2_rpwm, motor2_lpwm, motor2_ren, motor2_len]:
    GPIO.setup(pin, GPIO.OUT)

# Create PWM objects for speed control
# Frequency of 100 Hz is typically good for DC motors
motor1_forward_pwm = GPIO.PWM(motor1_rpwm, 100)
motor1_reverse_pwm = GPIO.PWM(motor1_lpwm, 100)
motor2_forward_pwm = GPIO.PWM(motor2_rpwm, 100)
motor2_reverse_pwm = GPIO.PWM(motor2_lpwm, 100)

# Start all PWM with 0% duty cycle (motors stopped)
motor1_forward_pwm.start(0)
motor1_reverse_pwm.start(0)
motor2_forward_pwm.start(0)
motor2_reverse_pwm.start(0)

def forward(speed):
    """
    Drive both motors forward at specified speed
    Speed should be between 0 (stopped) and 100 (full speed)
    """
    # Enable forward direction on both motors
    GPIO.output(motor1_ren, GPIO.HIGH)
    GPIO.output(motor1_len, GPIO.LOW)
    GPIO.output(motor2_ren, GPIO.HIGH)
    GPIO.output(motor2_len, GPIO.LOW)
    
    # Set forward PWM duty cycle for both motors
    motor1_forward_pwm.ChangeDutyCycle(speed)
    motor2_forward_pwm.ChangeDutyCycle(speed)
    
    # Ensure reverse PWM is zero
    motor1_reverse_pwm.ChangeDutyCycle(0)
    motor2_reverse_pwm.ChangeDutyCycle(0)

def stop():
    """Stop both motors"""
    motor1_forward_pwm.ChangeDutyCycle(0)
    motor1_reverse_pwm.ChangeDutyCycle(0)
    motor2_forward_pwm.ChangeDutyCycle(0)
    motor2_reverse_pwm.ChangeDutyCycle(0)
    
    # Disable both directions on both motors
    GPIO.output(motor1_ren, GPIO.LOW)
    GPIO.output(motor1_len, GPIO.LOW)
    GPIO.output(motor2_ren, GPIO.LOW)
    GPIO.output(motor2_len, GPIO.LOW)

try:
    # Drive motors forward continuously at 70% speed
    # (adjust speed percentage as needed)
    forward(70)
    
    # Keep the program running
    print("Motors running forward. Press CTRL+C to stop.")
    while True:
        time.sleep(0.1)  # Small delay to prevent CPU hogging
        
except KeyboardInterrupt:
    print("Stopping motors")
    stop()
    GPIO.cleanup()  # Clean up GPIO on CTRL+C exit

finally:
    GPIO.cleanup()  # Clean up GPIO on normal exit
