

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

class Drive:
    def __init__(self):
        # Enable pins as simple digital outputs
        self.left_l_enable = OutputDevice(LEFT_L_EN)
        self.left_r_enable = OutputDevice(LEFT_R_EN)
        self.right_l_enable = OutputDevice(RIGHT_L_EN)
        self.right_r_enable = OutputDevice(RIGHT_R_EN)

        # Set up PWM pins for speed control
        # Forward PWM pins get PWMOutputDevice for variable speed
        self.left_forward = PWMOutputDevice(LEFT_RPWM)
        self.right_forward = PWMOutputDevice(RIGHT_RPWM)

        # Reverse PWM pins set as regular outputs (we'll keep them off)
        self.left_reverse = OutputDevice(LEFT_LPWM)
        self.right_reverse = OutputDevice(RIGHT_LPWM)

    def forward(self, speed):
        left_l_enable.on()
        left_r_enable.on()
        right_l_enable.on()
        right_r_enable.on()
    
        # Ensure reverse pins are off
        left_reverse.off()
        right_reverse.off()

        left_forward.value = speed
        right_forward.value = speed

    # Function to stop motors and clean up
    def stop_motors(self):
        print("Stopping motors and cleaning up...")
        # Stop PWM
        self.left_forward.value = 0
        self.right_forward.value = 0

        # Disable enable pins
        self.left_l_enable.off()
        self.left_r_enable.off()
        self.right_l_enable.off()
        self.right_r_enable.off()

        # Ensure reverse pins are off
        self.left_reverse.off()
        self.right_reverse.off()
        print("Motors stopped")

    
"""    
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
        motor_speed = 0.3 # 60% duty cycle
        
        print(f"Running motors forward at {motor_speed*100}% speed for 5 seconds...")
        left_forward.value = motor_speed
        right_forward.value = 0.0


        
        # Run for 5 seconds
        time.sleep(5)
        
        # Stop motors
        stop_motors()
"""     
