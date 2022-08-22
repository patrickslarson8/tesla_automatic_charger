#Libraries
import RPi.GPIO as GPIO
import time
import math

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

GPIO.setup(TRIGGER, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(ARM_DIRECTION, GPIO.OUT)
GPIO.setup(ARM_SPEED, GPIO.OUT)
GPIO.setup(H_DIRECTION, GPIO.OUT)
GPIO.setup(H_SPEED, GPIO.OUT)
GPIO.setup(L_LIMIT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(R_LIMIT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SERVO, GPIO.OUT)

GPIO.output(ARM_DIRECTION, False)
a = GPIO.PWM(ARM_SPEED, 100)
a.start(0)
GPIO.output(H_DIRECTION, False)
h = GPIO.PWM(H_SPEED, 5000)
h.start(0)
s = GPIO.PWM(SERVO, 200)
s.start(0)

CLOSE_DISTANCE = 10 #The distance in CM that the extend command will stop at to be followed by the plug in routine
ARM_MAX_SPEED = 100
H_MAX_SPEED = 24.4

def extend():
    print("extending")
    GPIO.output(H_DIRECTION, False)
    GPIO.output(ARM_DIRECTION, False)
    start_time = time.time()
    while (get_distance() > CLOSE_DISTANCE) and (GPIO.input(R_LIMIT) == False) and (GPIO.input(L_LIMIT) == True):
        time_taken = (time.time() - start_time)
        curr_angle = time_taken * 6.35
        radians = math.radians(curr_angle)
        print(curr_angle)
        h_speed = H_MAX_SPEED*math.sin(radians)
        print(h_speed)
        a.ChangeDutyCycle(ARM_MAX_SPEED)
        if h_speed > 0.3:
            h.ChangeDutyCycle(h_speed)
        time.sleep(0.05)
    a.ChangeDutyCycle(0)
    h.ChangeDutyCycle(0)
    return

def retract():
    print("retracting")
    GPIO.output(H_DIRECTION, True)
    GPIO.output(ARM_DIRECTION, True)
    while (GPIO.input(L_LIMIT) == False) and (GPIO.input(R_LIMIT) == True):
        a.ChangeDutyCycle(ARM_MAX_SPEED)
        #h.ChangeDutyCycle(H_MAX_SPEED)
        time.sleep(0.01)
    a.ChangeDutyCycle(0)
    h.ChangeDutyCycle(0)
    return


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
            if (GPIO.input(L_LIMIT) == False) and (GPIO.input(R_LIMIT) == True):
                print("Left Limit Switch Triggered")
                retract()
            elif (GPIO.input(L_LIMIT) == True) and (GPIO.input(R_LIMIT) == False):
                print("Right Limit Switch Triggered")
                extend()


        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
        GPIO.cleanup()
