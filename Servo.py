import time
import os
os.system ("sudo pigpiod")
time.sleep(1)
import pigpio 

servoPin = 16

pi = pigpio.pi() 



def setHopperServoHome():
    pi.set_servo_pulsewidth(servoPin,1900)
    time.sleep(0.2)
    pi.set_servo_pulsewidth(servoPin,0)

def activateHopper():
    pi.set_servo_pulsewidth(servoPin,800)
    time.sleep(0.2)
    pi.set_servo_pulsewidth(servoPin,1900)
    time.sleep(0.25)
    pi.set_servo_pulsewidth(servoPin,0)



setHopperServoHome()
activateHopper()

pi.stop()


