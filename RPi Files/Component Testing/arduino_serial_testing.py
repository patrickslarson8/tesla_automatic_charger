import pigpio
import time
import serial

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
#pi.set_servo_pulsewidth(SERVO, 800)

if pi.read(L_LIMIT) == 1:
    arm_left_limit = False
else:
    arm_left_limit = True

if pi.read(R_LIMIT) == 1:
    arm_right_limit = False
else:
    arm_right_limit = True

def l_limit_activate(gpio, level, tick):
    global arm_left_limit
    if level == 0:
        pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
        arm_left_limit = True
        time.sleep(0.01)
        print("Left Limit Reached.")
    elif level == 1:
        arm_left_limit = False

def r_limit_activate(gpio, level, tick):
    global arm_right_limit
    if level == 0:
        pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
        arm_left_limit = True
        time.sleep(0.01)
        print("Right Limit Reached.")
    elif level == 1:
        arm_left_limit = False

cb0 = pi.callback(L_LIMIT, pigpio.EITHER_EDGE, l_limit_activate)
cb1 = pi.callback(R_LIMIT, pigpio.EITHER_EDGE, r_limit_activate)




if __name__ == '__main__':
    try:
        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        ser.bytesize=serial.EIGHTBITS
        ser.parity=serial.PARITY_NONE
        ser.stopbits=serial.STOPBITS_ONE
        ser.xonxoff=1
        ser.rtscts=0
        ser.flush()
        time.sleep(0.5)
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                intline = int(line)
                print(line)


        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Reset Aborted by User")
        pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
        pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
        pi.set_servo_pulsewidth(SERVO, 0)
        pi.stop()
