import pigpio
import time

pi = pigpio.pi()

#set GPIO Pins
TRIGGER = 2
ECHO = 4
ARM_DIRECTION = 11
ARM_SPEED_PIN = 9
H_DIRECTION = 10
H_SPEED_PIN = 13
SERVO = 26
L_LIMIT = 20
R_LIMIT = 21

MOTOR_SPEED = 350000

#set GPIO inputs/outputs
pi.set_mode(TRIGGER, pigpio.OUTPUT)
pi.set_mode(ECHO, pigpio.INPUT)
pi.set_pull_up_down(ECHO, pigpio.PUD_DOWN)
pi.set_mode(ARM_DIRECTION, pigpio.OUTPUT)
pi.set_mode(ARM_SPEED_PIN, pigpio.OUTPUT)
pi.set_mode(H_DIRECTION, pigpio.OUTPUT)
pi.set_mode(L_LIMIT, pigpio.INPUT)
pi.set_pull_up_down(L_LIMIT, pigpio.PUD_UP)
pi.set_mode(R_LIMIT, pigpio.INPUT)
pi.set_pull_up_down(R_LIMIT, pigpio.PUD_UP)
pi.set_mode(SERVO, pigpio.OUTPUT)

#set PWM modes

pi.set_PWM_frequency(ARM_SPEED_PIN, 500)
pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
pi.set_servo_pulsewidth(SERVO, 1400)

# The servo reponds to changes as small as 10ms, so
# for the pi.set_servo_pulsewidth command that equates to
# increments of 10.
# Low limit (counter-clockwise) is 500
# High limit (clockwise) is 2500 but seems to be less precise than 500 - 1500
# May just be a phenomenon of the load of the handle
# Useful range is 1300 - 1660
# plug-in angle is about 1400
# parked is 1660
# a good sleep time for max speed is 0.015


if __name__ == '__main__':
    try:
        x = 1520
        flag = 0
        while True:
            # if flag == 0:
            #     x = (x-10)
            #     if x < 1300:
            #         flag = 1
            # else:
            #     x = (x + 10)
            #     if x > 1770:
            #         flag = 0
            pi.set_servo_pulsewidth(SERVO, x)
            print(x)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Reset Aborted by User")
        pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
        pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
        pi.write(SERVO, 0)
        pi.stop()
