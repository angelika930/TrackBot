import gpiod
import time
import threading
import atexit
import sys

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
        try:
            self.chip = gpiod.Chip('gpiochip0')
            self.lines = {}  # Dictionary to store configured GPIO lines
            
            # Configure all GPIO pins
            self._setup_gpio()
            
            # PWM threads
            self.pwm_threads = []
            self.pwm_running = True
            
            # Set up clean exit
            atexit.register(self.cleanup)
            
        except Exception as e:
            print(f"Error initializing GPIO: {e}")
            print("\nIf it's a permission error, make sure you're running with sudo.")
            print("If it's an API error, you may need to check your gpiod version.")
            sys.exit(1)
    
    def _setup_gpio(self):
        """Configure all GPIO pins using gpiod"""
        for pin in self.all_pins:
            try:
                # Request line for output - using the updated API
                line = self.chip.get_line(pin)
                
                # The API for setting up lines varies between gpiod versions
                # Try different API approaches
                try:
                    # Newer API style
                    line.request(consumer="MotorControl", type=gpiod.LINE_REQ_DIR_OUT)
                except AttributeError:
                    try:
                        # Alternative API style
                        line.request(consumer="MotorControl", direction=gpiod.Line.DIRECTION_OUTPUT)
                    except AttributeError:
                        # Fallback for very new API
                        config = gpiod.LineRequest()
                        config.consumer = "MotorControl"
                        config.request_type = gpiod.LineRequest.DIRECTION_OUTPUT
                        line.request(config)
                
                # Store the configured line in our dictionary
                self.lines[pin] = line
                
                # Initialize to LOW (0)
                line.set_value(0)
                
            except Exception as e:
                print(f"Error setting up GPIO pin {pin}: {e}")
                print("This could be due to an incompatible gpiod version.")
                raise
    
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


# Alternative simpler implementation using RPi.GPIO
def run_with_rpi_gpio():
    """
    Fallback function that uses RPi.GPIO instead of gpiod
    This is a simpler implementation that might be more reliable
    """
    import RPi.GPIO as GPIO
    
    # Set up GPIO mode
    GPIO.setmode(GPIO.BCM)
    
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
        GPIO.output(pin, GPIO.LOW)
    
    # Create PWM objects for speed control
    # Frequency of 500 Hz works well for DC motors
    motor1_forward_pwm = GPIO.PWM(motor1_rpwm, 500)
    motor1_reverse_pwm = GPIO.PWM(motor1_lpwm, 500)
    motor2_forward_pwm = GPIO.PWM(motor2_rpwm, 500)
    motor2_reverse_pwm = GPIO.PWM(motor2_lpwm, 500)
    
    # Start all PWM with 0% duty cycle (motors stopped)
    motor1_forward_pwm.start(0)
    motor1_reverse_pwm.start(0)
    motor2_forward_pwm.start(0)
    motor2_reverse_pwm.start(0)
    
    try:
        # Enable forward direction on both motors
        GPIO.output(motor1_ren, GPIO.HIGH)
        GPIO.output(motor1_len, GPIO.LOW)
        GPIO.output(motor2_ren, GPIO.HIGH)
        GPIO.output(motor2_len, GPIO.LOW)
        
        # Set forward PWM duty cycle for both motors (70%)
        motor1_forward_pwm.ChangeDutyCycle(70)
        motor2_forward_pwm.ChangeDutyCycle(70)
        
        print("Motors running forward at 70% speed")
        print("Press CTRL+C to stop motors")
        
        # Keep the program running
        while True:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        print("Stopping motors")
        motor1_forward_pwm.ChangeDutyCycle(0)
        motor1_reverse_pwm.ChangeDutyCycle(0)
        motor2_forward_pwm.ChangeDutyCycle(0)
        motor2_reverse_pwm.ChangeDutyCycle(0)
        
        # Disable both directions on both motors
        GPIO.output(motor1_ren, GPIO.LOW)
        GPIO.output(motor1_len, GPIO.LOW)
        GPIO.output(motor2_ren, GPIO.LOW)
        GPIO.output(motor2_len, GPIO.LOW)
        
    finally:
        # Clean up
        GPIO.cleanup()
        print("GPIO cleaned up")


if __name__ == "__main__":
    try:
        # Try the gpiod implementation first
        print("Attempting to use gpiod library...")
        motor_controller = MotorController()
        motor_controller.start_pwm()
        motor_controller.forward(70, 70)
        
        # Keep the program running
        print("Press CTRL+C to stop motors")
        while True:
            time.sleep(0.1)
            
    except Exception as e:
        # If gpiod fails, fall back to RPi.GPIO
        print(f"Error with gpiod: {e}")
        print("Falling back to RPi.GPIO implementation...")
        run_with_rpi_gpio()
    
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        if 'motor_controller' in locals():
            motor_controller.stop()
