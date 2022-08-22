import pigpio
import time


pi = pigpio.pi()

#set GPIO Pins
TRIGGER = 2
ECHO = 4
ARM_DIRECTION = 11
ARM_SPEED = 9
H_DIRECTION = 10
H_SPEED = 13
SERVO = 26
L_LIMIT = 20
R_LIMIT = 21

MOTOR_SPEED = (250000) #with conversion to pigpio this is now 0-255 so it might seem like it's going slowly

pi.set_mode(TRIGGER, pigpio.OUTPUT)
pi.set_mode(ECHO, pigpio.INPUT)
pi.set_pull_up_down(ECHO, pigpio.PUD_DOWN)
pi.set_mode(ARM_DIRECTION, pigpio.OUTPUT)
pi.set_mode(ARM_SPEED, pigpio.OUTPUT)
pi.set_mode(H_DIRECTION, pigpio.OUTPUT)
pi.set_mode(L_LIMIT, pigpio.INPUT)
pi.set_pull_up_down(L_LIMIT, pigpio.PUD_UP)
pi.set_mode(R_LIMIT, pigpio.INPUT)
pi.set_pull_up_down(R_LIMIT, pigpio.PUD_UP)
pi.set_mode(SERVO, pigpio.OUTPUT)

#set PWM modes
pi.set_PWM_frequency(ARM_SPEED, 500)
pi.set_PWM_dutycycle(ARM_SPEED, 0)
pi.hardware_PWM(H_SPEED, 20000, 0)
pi.set_servo_pulsewidth(SERVO, 1500)
print(pi.get_PWM_frequency(H_SPEED))
print(pi.get_mode(H_SPEED))

def get_distance():
    # set Trigger to HIGH for 10 ms
    pi.gpio_trigger(TRIGGER, 10, 1)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while pi.read(ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while pi.read(ECHO) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    if distance > 0:
        return distance
    else:
        return 0

if __name__ == '__main__':
    try:
        while True:
            dist = get_distance()
            print ("Measured Distance = %.1f cm" % dist)
            if (pi.read(L_LIMIT) == 0) and (pi.read(R_LIMIT) == 1):
                print("Left Limit Switch Triggered")
                pi.write(H_DIRECTION, 0)
                pi.hardware_PWM(H_SPEED, 20000, MOTOR_SPEED)
            elif (pi.read(L_LIMIT) == 1) and (pi.read(R_LIMIT) == 0):
                print("Right limit switch triggered")
                pi.write(H_DIRECTION, 1)
                pi.hardware_PWM(H_SPEED, 20000, MOTOR_SPEED)
                print(pi.get_PWM_frequency(H_SPEED))
                print(pi.get_PWM_dutycycle(H_SPEED))
            #elif (dist < 250):
                #pi.write(ARM_DIRECTION, 1)
                #pi.set_PWM_dutycycle(ARM_SPEED, (255-dist))
            else:
                pi.hardware_PWM(H_SPEED, 20000, 0)
            time.sleep(0.1) #polling any faster than .05 seems to cause issues


        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
        pi.set_PWM_dutycycle(ARM_SPEED, 0)
        pi.set_servo_pulsewidth(SERVO, 0)
        pi.stop()
