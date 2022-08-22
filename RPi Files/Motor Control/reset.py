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

MOTOR_SPEED = 10000

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
pi.set_servo_pulsewidth(SERVO, 1500)



if __name__ == '__main__':
    try:
        parked = False
        message = False
        while True:
            if (pi.read(R_LIMIT) == 1) & (parked == False):
                if message == False:
                    print ("Parking Horizontal Position")
                    message = True
                pi.write(H_DIRECTION, 1)
                pi.hardware_PWM(H_SPEED_PIN, 20000, MOTOR_SPEED)
            elif parked == True:
                print("Retracting Arm")
                pi.write(ARM_DIRECTION, 1)
                pi.set_PWM_dutycycle(ARM_SPEED_PIN, 100)
                time.sleep(15)
                print("Finished, exiting...")
                pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
                exit()
            elif pi.read(R_LIMIT) == 0:
                print("Parked")
                pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
                parked = True
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Reset Aborted by User")
        pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
        pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
        pi.set_servo_pulsewidth(SERVO, 0)
        pi.stop()
