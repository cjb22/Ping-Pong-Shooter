import RPi.GPIO as GPIO
import time
import sys

#Define The Stepper Control Pins
StepPin = 19
DirectionPin = 26

#Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(StepPin,GPIO.OUT)
GPIO.setup(DirectionPin, GPIO.OUT)


StepPauseTime = 0.0005

    
def PreciseTurn( dir, steps):
    if dir == "CW":
        GPIO.output(DirectionPin,GPIO.LOW)
    else:
        GPIO.output(DirectionPin,GPIO.HIGH)

    for i in range(steps):
        GPIO.output(StepPin, GPIO.HIGH)
        time.sleep(StepPauseTime)
        GPIO.output(StepPin, GPIO.LOW)
        time.sleep(StepPauseTime)
    return



if __name__ == "__main__":
    if len(sys.argv) == 3:
        dir = sys.argv[1]
        steps = int(sys.argv[2])
        if (dir == "CW" or dir == "CCW"):
            if (steps > 0 and steps <= 2000):
                PreciseTurn(dir, steps)
            else: print("Amount of steps not valid. (0 - 2000)")
        else:print ("Direction not valid. (CW or CCW)")

    else:
        print("MoveStepper test use case (R-L-R)")
        PreciseTurn("CW",250)
        PreciseTurn("CCW",500)
        PreciseTurn("CW",250)
                    

    
GPIO.cleanup()       
