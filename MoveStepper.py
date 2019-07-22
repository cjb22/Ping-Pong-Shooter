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

    
def turnSteps( dir, steps):
    if dir == "CW":
        GPIO.output(DirectionPin,GPIO.LOW)
    else:
        GPIO.output(DirectionPin,GPIO.HIGH)

    for i in range(steps):
        GPIO.output(StepPin, GPIO.HIGH)
        time.sleep(StepPauseTime)
        GPIO.output(StepPin, GPIO.LOW)
        time.sleep(StepPauseTime)


# Function to turn the stepper motor, which is connected to a bigger gear
bigToSmallRatio = 123/25.           # Gear ratio
degreesPerStep = 1.8 / 8            # Using an 1.8 degree stepper motor, operating at 1/8th steps, fill in 1.8 / 8
def turnDegrees( degrees):

    # Check the direction to turn
    if degrees < 0:
        GPIO.output(DirectionPin,GPIO.HIGH)     # CCW
        degrees = degrees * -1
    else:
        GPIO.output(DirectionPin,GPIO.LOW)      # CW

    # Determine the amount of steps to turn by
    smallGearToTurn = bigToSmallRatio * degrees
    stepsToTake  = smallGearToTurn / degreesPerStep

    # Turn the stepper
    for i in range(int(stepsToTake)):
        GPIO.output(StepPin, GPIO.HIGH)
        time.sleep(StepPauseTime)
        GPIO.output(StepPin, GPIO.LOW)
        time.sleep(StepPauseTime)
    


if __name__ == "__main__":
    if len(sys.argv) == 3:
        dir = sys.argv[1]
        steps = int(sys.argv[2])
        if (dir == "CW" or dir == "CCW"):
            if (steps > 0 and steps <= 2000):
                turnSteps(dir, steps)
            else: print("Amount of steps not valid. (0 - 2000)")
        else:print ("Direction not valid. (CW or CCW)")

    elif len(sys.argv) == 2:
        degrees = float(sys.argv[1])
        if degrees > -360 and degrees < 360:
            turnDegrees(degrees)
        else: print("Degrees number not valid. (-360 - 360)")
        
        

    else:
        print("MoveStepper test use case (R-L-R)")
        turnSteps("CW",250)
        turnSteps("CCW",500)
        turnSteps("CW",250)
                    

    
GPIO.cleanup()       
