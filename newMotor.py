import gpiod
import time
import threading
import atexit

class MotorController:
    def __init__(self):
        # Motor 1 Pin Definitions
        self.motor1_rpwm_pin = 12  # RPWM pin for motor controller 1
        self.motor1_lpwm_pin = 13  # LPWM pin for motor controller 1
        self.motor1_ren_pin = 5    # R_EN pin for motor controller 1
        self.motor1_len_pin = 6    # L_EN pin for motor controller 1

        # Motor 2 Pin Definitions
        self.motor2_rpwm_pin = 16  # RPWM pin for motor controller 2
        self.motor2_lpwm_pin = 20  # LPWM pin for motor controller 2
        self.motor2_ren_pin = 21   # R_EN pin for motor controller 2
        self.motor2_len_pin = 26   # L_EN pin for motor controller 2

        # All motor pins in one list for easy setup
        self.all_pins = [
            self.motor1_rpwm_pin, self.motor1_lpwm_pin, self.motor1_ren_pin, self.motor1_len_pin,
            self.motor2_rpwm_pin, self.motor2_lpwm_pin, self.motor2_ren_pin, self.motor2_len_pin
        ]
        
        # PWM frequencies and duty cycles
        self.pwm_freq = 500  # PWM frequency in Hz
        self.motor1_forward_duty = 0
        self.motor1_reverse_duty = 0
        self.motor2_forward_duty = 0
        self.motor2_reverse_duty = 0
        
        # Initialize gpiod
        self.chip = gpiod.chip("gpiochip0")  # The main GPIO chip on Raspberry Pi
        self.lines = {}  # Dictionary to store configured GPIO lines
        
        # Configure all GPIO pins
        self._setup_gpio()
        
        # PWM threads
        self.pwm_threads = []
        self.pwm_running = True
        
        # Set up clean exit
        atexit.register(self.cleanup)
    
    def _setup_gpio(self):
        """Configure all GPIO pins using gpiod"""
        for pin in self.all_pins:
            # Request line for output
            line = self.chip.get_line(pin)
            config = gpiod.line_request()
            config.consumer = "MotorControl"
            config.request_type = gpiod.line_request.DIRECTION_OUTPUT
            line.request(config)
            
            # Store the configured line in our dictionary
            self.lines[pin] = line
            
            # Initialize to LOW (0)
            line.set_value(0)
    
    def _pwm_thread(self, pin, duty_cycle_ref):
        """Thread function to generate PWM signal on a pin"""
        line = self.lines[pin]
        period = 1.0 / self.pwm_freq
        
        while self.pwm_running:
            # Get the current duty cycle (may be updated from main thread)
            duty = getattr(self, duty_cycle_ref)
            
            if duty > 0:
                # Calculate on and off times
                on_time = period * (duty / 100.0)
                off_time = period - on_time
                
                # Generate PWM cycle
                line.set_value(1)
                time.sleep(on_time)
                
                if off_time > 0:
                    line.set_value(0)
                    time.sleep(off_time)
            else:
                # If duty cycle is 0, just keep the pin LOW
                line.set_value(0)
                time.sleep(period)
    
    def start_pwm(self):
        """Start PWM threads for all motors"""
        # Create and start a PWM thread for each motor control pin
        pwm_configs = [
            (self.motor1_rpwm_pin, "motor1_forward_duty"),
            (self.motor1_lpwm_pin, "motor1_reverse_duty"),
            (self.motor2_rpwm_pin, "motor2_forward_duty"),
            (self.motor2_lpwm_pin, "motor2_reverse_duty")
        ]
        
        for pin, duty_ref in pwm_configs:
            thread = threading.Thread(
                target=self._pwm_thread,
                args=(pin, duty_ref),
                daemon=True
            )
            thread.start()
            self.pwm_threads.append(thread)
    
    def forward(self, speed1=70, speed2=70):
        """Drive both motors forward at specified speeds"""
        # Enable forward direction on both motors
        self.lines[self.motor1_ren_pin].set_value(1)
        self.lines[self.motor1_len_pin].set_value(0)
        self.lines[self.motor2_ren_pin].set_value(1)
        self.lines[self.motor2_len_pin].set_value(0)
        
        # Set duty cycles for forward direction
        self.motor1_forward_duty = speed1
        self.motor1_reverse_duty = 0
        self.motor2_forward_duty = speed2
        self.motor2_reverse_duty = 0
        
        print(f"Motors running forward: Motor 1 at {speed1}%, Motor 2 at {speed2}%")
    
    def stop(self):
        """Immediately stop both motors"""
        # Set all PWM duty cycles to 0
        self.motor1_forward_duty = 0
        self.motor1_reverse_duty = 0
        self.motor2_forward_duty = 0
        self.motor2_reverse_duty = 0
        
        # Disable both directions on both motors
        for pin in [self.motor1_ren_pin, self.motor1_len_pin, 
                    self.motor2_ren_pin, self.motor2_len_pin]:
            self.lines[pin].set_value(0)
        
        print("Motors stopped")
    
    def cleanup(self):
        """Clean up GPIO and threads on exit"""
        print("Cleaning up...")
        
        # Stop the PWM threads
        self.pwm_running = False
        time.sleep(0.1)  # Give threads time to exit
        
        # Release all GPIO lines
        for line in self.lines.values():
            if line:
                line.release()
        
        print("Cleanup complete")


if __name__ == "__main__":
    motor_controller = MotorController()
    
    try:
        # Start PWM generation
        motor_controller.start_pwm()
        
        # Drive motors forward continuously
        motor_controller.forward(70, 70)
        
        # Keep the program running
        print("Press CTRL+C to stop motors")
        while True:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        motor_controller.stop()
