from gpiozero import OutputDevice, PWMOutputDevice
import time
import sys
from signal import pause

# Using BCM GPIO pin numbers (converted from physical pins)
# Left motor pins
LEFT_L_EN = 17    # BCM GPIO for physical pin 11
LEFT_R_EN = 18    # BCM GPIO for physical pin 12
LEFT_RPWM = 5     # BCM GPIO for physical pin 29 (Forward PWM)
LEFT_LPWM = 27    # BCM GPIO for physical pin 13 (Reverse PWM)

# Right motor pins
RIGHT_R_EN = 23   # BCM GPIO for physical pin 16
RIGHT_L_EN = 24   # BCM GPIO for physical pin 18
RIGHT_RPWM = 25   # BCM GPIO for physical pin 22 (Forward PWM)
RIGHT_LPWM = 12   # BCM GPIO for physical pin 32 (Reverse PWM)

# Set up pins using gpiozero
print("Setting up motor pins...")

try:
    # Enable pins as simple digital outputs
    left_l_enable = OutputDevice(LEFT_L_EN)
    left_r_enable = OutputDevice(LEFT_R_EN)
    right_l_enable = OutputDevice(RIGHT_L_EN)
    right_r_enable = OutputDevice(RIGHT_R_EN)
    
    # Set up PWM pins for speed control
    # Forward PWM pins get PWMOutputDevice for variable speed
    left_forward = PWMOutputDevice(LEFT_RPWM)
    right_forward = PWMOutputDevice(RIGHT_RPWM)
    
    # Reverse PWM pins set as regular outputs (we'll keep them off)
    left_reverse = OutputDevice(LEFT_LPWM)
    right_reverse = OutputDevice(RIGHT_LPWM)
    
    print("All pins configured successfully")
    
    # Function to stop motors and clean up
    def stop_motors():
        print("Stopping motors and cleaning up...")
        # Stop PWM
        left_forward.value = 0
        right_forward.value = 0
        
        # Disable enable pins
        left_l_enable.off()
        left_r_enable.off()
        right_l_enable.off()
        right_r_enable.off()
        
        # Ensure reverse pins are off
        left_reverse.off()
        right_reverse.off()
        
        print("Motors stopped")
    
    # Main program
    try:
        # Enable both motors
        print("Enabling motors...")
        left_l_enable.on()
        left_r_enable.on()
        right_l_enable.on()
        right_r_enable.on()
        
        # Make sure reverse direction is off
        left_reverse.off()
        right_reverse.off()
        
        # Set forward speed (0.0 to 1.0, where 1.0 is 100% duty cycle)
        motor_speed = 0.6  # 60% duty cycle
        
        print(f"Running motors forward at {motor_speed*100}% speed for 5 seconds...")
        left_forward.value = motor_speed
        right_forward.value = motor_speed
        
        # Run for 5 seconds
        time.sleep(5)
        
        # Stop motors
        stop_motors()
        
    except KeyboardInterrupt:
        print("Program stopped by user")
        stop_motors()
    
    except Exception as e:
        print(f"Error during operation: {e}")
        stop_motors()
    
except Exception as e:
    print(f"Error setting up pins: {e}")
    sys.exit(1)