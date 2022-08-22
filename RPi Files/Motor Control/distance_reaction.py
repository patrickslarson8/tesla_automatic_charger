#Libraries
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

#set GPIO Pins
TRIGGER = 2
ECHO = 4
ARM_DIRECTION = 11
ARM_SPEED = 9
H_DIRECTION = 10
H_SPEED = 22
SERVO = 26
L_LIMIT = 20
R_LIMIT = 21

MOTOR_SPEED = 50

#set GPIO inputs/outputs
GPIO.setup(TRIGGER, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(ARM_DIRECTION, GPIO.OUT)
GPIO.setup(ARM_SPEED, GPIO.OUT)
GPIO.setup(H_DIRECTION, GPIO.OUT)
GPIO.setup(H_SPEED, GPIO.OUT)
GPIO.setup(L_LIMIT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(R_LIMIT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SERVO, GPIO.OUT)

#set PWM modes
#a is arm
#h is horizontal
#s is servo
GPIO.output(ARM_DIRECTION, False)
a = GPIO.PWM(ARM_SPEED, 100)
a.start(0)
GPIO.output(H_DIRECTION, False)
h = GPIO.PWM(H_SPEED, 500)
h.start(0)
s = GPIO.PWM(SERVO, 80)
s.start(0)

def get_distance():
    # set Trigger to HIGH
    GPIO.output(TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(ECHO) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance

if __name__ == '__main__':
    try:
        while True:
            dist = get_distance()
            print ("Measured Distance = %.1f cm" % dist)
            if (GPIO.input(L_LIMIT) == False) and (GPIO.input(R_LIMIT) == True):
                print("Left Limit Switch Triggered")
                GPIO.output(H_DIRECTION, False)
                h.ChangeDutyCycle(MOTOR_SPEED)
            elif (GPIO.input(L_LIMIT) == True) and (GPIO.input(R_LIMIT) == False):
                print("Right limit switch triggered")
                GPIO.output(H_DIRECTION, True)
                h.ChangeDutyCycle(MOTOR_SPEED)
            elif (dist < 100):
                GPIO.output(ARM_DIRECTION, True)
                a.ChangeDutyCycle(50-dist)
            else:
                h.ChangeDutyCycle(0)
            time.sleep(0.1) #polling any faster than .05 seems to cause issues
            #for x in range(7,48):
            #    print(x)
            #    s.ChangeDutyCycle(x)
            #    time.sleep(1)

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
