import os     
import time    
os.system ("sudo pigpiod")
time.sleep(1)
import pigpio

pi = pigpio.pi() 
escPin = 20


def activateESC():
    #Set maximum and minimum PWM
    pi.set_servo_pulsewidth(escPin, 2500)
    time.sleep(1)
    pi.set_servo_pulsewidth(escPin, 500) 
    time.sleep(1)

    #Fire the ESC 
    pi.set_servo_pulsewidth(escPin, 1700)
    time.sleep(2)

    


activateESC()

pi.set_servo_pulsewidth(escPin, 0) # Stop servo pulses.
pi.stop() # Disconnect pigpio.
