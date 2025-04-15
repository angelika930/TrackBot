import RPi.GPIO as GPIO

class motor:
   def __init__(self):
  
      #use pin numbers on pi board
      GPIO.setmode(GPIO.BOARD)

      #alert us if pin is configured to something other than default
      GPIO.setwarnings(False)

      #pinout for right motor controller
      #EN = enable left or right
      self.R_EN = 29
      self.R_PWM = 33

      #pinout for left motor controller
      self.L_EN = 31
      self.L_PWM = 32
      
      #check in case pin numbers wrong
      GPIO.setup(self.R_EN, GPIO.OUT)
      GPIO.setup(self.R_PWM, GPIO.OUT)
      GPIO.setup(self.L_EN, GPIO.OUT)
      GPIO.setup(self.L_PWM, GPIO.OUT)
      GPIO.output(self.R_EN, True)
      GPIO.output(self.L_EN, True)

      
      def neutral(self):
         GPIO.output(self.R_PWM, False)
         GPIO.output(self.L_PWM, False)

      def right(self):
         GPIO.output(self.L_PWM, False)
         GPIO.output(self.R_PWM, True)

      def left(self):
         GPIO.output(self.L_PWM, True)
         GPIO.output(self.R_PWM, False)

      def forward(self):
         GPIO.output(self.L_PWM, True)
         GPIO.output(self.R_PWM, True)

forward()
