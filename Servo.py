import pigpio
import time

pi = pigpio.pi() 



def setHopperServoHome():
    pi.set_servo_pulsewidth(21,1900)
    time.sleep(0.2)
    pi.set_servo_pulsewidth(21,0)

def activateHopper():
    pi.set_servo_pulsewidth(21,800)
    time.sleep(0.2)
    pi.set_servo_pulsewidth(21,1900)
    time.sleep(0.25)
    pi.set_servo_pulsewidth(21,0)



setHopperServoHome()
activateHopper()


