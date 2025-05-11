from gpiozero import OutputDevice, PWMOutputDevice
import time

#Motor 1 pins
in1 = OutputDevice(17)
in2 = OutputDevice(18)
ena = PWMOutputDevice(22)

#Motor 2 pins
in3 = OutputDevice(23)
in4 = OutputDevice(24)
enb = PWMOutputDevice(25)

def forward():
   in1.on()
   in2.off()
   in3.on()
   in4.off()
   ena.value = 0.5
   enb.value = 0.5

def stop():
   ena.value = 0.0
   enb.value = 0.0
   in1.off()
   in2.off()
   in3.off()
   in4.off()
   

forward()
sleep(5)
stop()
