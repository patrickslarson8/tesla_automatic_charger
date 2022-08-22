## import stuffs
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import pigpio
import io
import re
import time
import math
import statistics
from annotation import Annotator
import numpy as np
import picamera
from PIL import Image
from tflite_runtime.interpreter import Interpreter
import teslapy
from teslapy import Tesla
import paho.mqtt.client as paho
import board
import serial
import adafruit_us100


## Configurable Tuning Parameters
broker="192.168.1.202"
port=1883
detect_distance = 90 ## The distance in cm if less than will start plug finding
really_close_distance = 20 ## How close it will get after opening charge port door
idle_sleep_time = 60 ## Seconds between re-checking if LSP is home
port_detect_distance = 40 ## The distance in cm if less than will move to before looking for port
plug_in_offset = 30 ## Distance will move to before trying to plug in. Used in math for charger rotation mechanism
centering_speed = 200000
centering_step_size = 30 # multiplier to figure out time in sec arm will move at centering speed
x_offset = 0
y_offset = 0
score_threshold = 0.45
model_path = "/home/pi/Desktop/resnet_tf1_2/detect.tflite"
label_path = "/home/pi/Desktop/annotations/label_lite.txt"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 640
search_step_size = 5 ## Amount of seconds(?) that arm will move when finding reflector
search_move_speed = 250000 # weird hardware_PWM thing so it's out of like a million
rotate_clearance_distance = 18
servo_position = 800 #variable to hold servo position but changing this will change start position
insert_distance = 15.5
insert_speed = 75 #out of 100 percent
logo_centering_speed = 125000
logo_centering_step_size = 18
logo_center_location = 0.57
logo_center_precision = 0.015
extract_distance = 30
insert_step_size = 0.7 #time that arm will move before taking another distance reading
insert_buffer = 2 #amount in cm that if close enough will stop pushing into the car

pi = pigpio.pi()

##pinmap
TRIGGER = 2 #change to tx and ux
ECHO = 4
ARM_DIRECTION_PIN = 11
ARM_SPEED_PIN = 9
H_DIRECTION_PIN = 10
H_SPEED_PIN = 13
SERVO = 26
L_LIMIT = 20
R_LIMIT = 21

## Prep the battlefield
pi.set_mode(TRIGGER, pigpio.OUTPUT)
pi.set_mode(ECHO, pigpio.INPUT)
pi.set_pull_up_down(ECHO, pigpio.PUD_DOWN)
pi.set_mode(ARM_DIRECTION_PIN, pigpio.OUTPUT)
pi.set_mode(ARM_SPEED_PIN, pigpio.OUTPUT)
pi.set_mode(H_DIRECTION_PIN, pigpio.OUTPUT)
pi.set_mode(L_LIMIT, pigpio.INPUT)
pi.set_pull_up_down(L_LIMIT, pigpio.PUD_UP)
pi.set_glitch_filter(L_LIMIT, 100)
pi.set_mode(R_LIMIT, pigpio.INPUT)
pi.set_pull_up_down(R_LIMIT, pigpio.PUD_UP)
pi.set_glitch_filter(R_LIMIT, 100)
pi.set_mode(SERVO, pigpio.OUTPUT)
#set PWM modes
pi.set_PWM_frequency(ARM_SPEED_PIN, 2000)
pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
pi.set_servo_pulsewidth(SERVO, servo_position)

## Callbacks for stopping the movement of the arm at the end of its travel
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

## Function to get ultrasonic distance. Uses rolling average of #SAMPLES readings
def get_distance():
    raw_dist = us100.distance
    return raw_dist



def main():
    ## Initialize, to include verifing current state
    uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=3000)
    # Create a US-100 module instance.
    global us100
    us100 = adafruit_us100.US100(uart)
    while True:
        print("-----")
        get_distance()
        time.sleep(0.5)

## Run when called as main
if __name__ == '__main__':
    try:
        time.sleep(3) #to let GPIO pins settle before doing anything
        main()
    except KeyboardInterrupt:
        print("Process stopped by User")
        pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
        pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
        pi.set_servo_pulsewidth(SERVO, 0)
        pi.stop()
