import os     
import time    
os.system ("sudo pigpiod")
time.sleep(1)
import pigpio

pi = pigpio.pi() 
escPin = 16


def activateESC():
    #Set maximum and minimum PWM
    pi.set_servo_pulsewidth(escPin, 2500)
    time.sleep(0.8)
    pi.set_servo_pulsewidth(escPin, 500) 
    time.sleep(0.8)

    #Fire the ESC 
    pi.set_servo_pulsewidth(escPin, 1900)
    time.sleep(3)

    


activateESC()

pi.set_servo_pulsewidth(escPin, 0) # Stop servo pulses.
pi.stop() # Disconnect pigpio.
