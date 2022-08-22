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
import serial
import traceback

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
arm_length = 77.8

pi = pigpio.pi()

##pinmap
TRIGGER = 2
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

## To get zigbee control of philips HUE light
def on_publish(client,userdata,result):             #create function for callback
    print("data published \n")
    pass

## Function to get ultrasonic distance. Uses rolling average of #SAMPLES readings
def get_distance(samples):
    distances = []
    for _ in range(samples):
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
        distance = (TimeElapsed * 34300) / 2
        distances.append(distance)
        time.sleep(0.1)
    avg_distance = sum(distances) / len(distances)
    if avg_distance > 0:
        print(avg_distance)
        return avg_distance
    else:
        return 0.

def coordinated_extension(requested_extension, start_angle_degree, arm_speed, arm_direction): # returns (Success, curr_angle_degree)
    ser.flush()
    print("Requested extension: {0:.2f}".format(requested_extension))
    curr_angle_degree = start_angle_degree
    arm_speed_comp = (1 - ((1 - (arm_speed/100)) / 2)) * 100
    timer_0 = 0
    curr_angle = math.radians(curr_angle_degree)
    est_extension = math.sin(curr_angle) * arm_length
    h_start = (arm_length - (math.cos(curr_angle) * arm_length))
    h_position = 0
    h_position_old = h_position = (arm_length - (math.cos(curr_angle) * arm_length)) - h_start
    global h_speed
    h_speed = 0 #Input power amount to motor
    h_speed_est = 0 #Calculated component velocty of arm in H-direction
    h_speed_measured = 0
    split_time_end = 0
    comp_speed = 1
    line = 0
    final_extension = requested_extension + est_extension
    angle_compensation = 1 - ((100-arm_speed)/25)/100
    encoder_measured_distance_old = 0
    encoder_measured_distance = 0
    print("Angle Compensation Factor: {0:.6f}".format(angle_compensation))
    if arm_direction == 0:
        pi.write(ARM_DIRECTION_PIN, 0)
        pi.write(H_DIRECTION_PIN, 0)
        if final_extension > 85:
            print("Requested extension too far, exiting...")
            return False, curr_angle_degree
    else:
        pi.write(ARM_DIRECTION_PIN, 1)
        pi.write(H_DIRECTION_PIN, 1)
    start_time_0 = time.time()
    pi.set_PWM_dutycycle(ARM_SPEED_PIN, arm_speed)
    while (est_extension < final_extension):
        split_time_start = time.time()
        ser.write(1)
        linestr = ser.readline().decode('utf-8').rstrip()
        try:
            line = float(linestr)
        except:
            pass
        encoder_measured_distance=(line*0.0012555)
        if arm_left_limit == True:
            pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
            pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
            print("Charge port out of reach in horizontal axis.")
            return False, curr_angle_degree
        else:
            h_speed_measured = (encoder_measured_distance_old-encoder_measured_distance)/(split_time_end-split_time_start)
            timer_0 = (time.time() - start_time_0)
            curr_angle_degree = (timer_0 * (arm_speed/100) * 2.04) + start_angle_degree + 1 #Adjust the time value here instead of angle to make work properly?
            curr_angle = math.radians(curr_angle_degree*angle_compensation)
            tangential_speed = (arm_speed/100) * 2.637053029
            h_speed_est = tangential_speed * math.sin(curr_angle)
            est_extension = math.sin(curr_angle) * arm_length
            comp_speed = (h_speed_est-h_speed_measured) * 15000
            h_position = (arm_length - (math.cos(curr_angle) * arm_length)) - h_start
            lag = h_position - encoder_measured_distance
            h_speed = int((h_speed_est*36000)) + 125000 + int(comp_speed)
            encoder_measured_distance_old = encoder_measured_distance
            if (lag > .3) and h_speed < 125000:
                h_speed = 175000
                pi.hardware_PWM(H_SPEED_PIN, 20000, h_speed)
            elif h_speed < 500000:
                pi.hardware_PWM(H_SPEED_PIN, 20000, h_speed)

            #pi.hardware_PWM(H_SPEED_PIN, 20000, h_speed)
            print("cur_angle: {0:.1f}  est_ext: {3:.1f}cm  h_pos: {4:.1f}cm  Cur_Time: {1:.1f}s  h_speed: {2:,}  enc_dist: {5:0.1f}cm  buff_wait: {6:,}  lag: {7:.1f} comp_speed: {8:.1f}  h_speed_est: {9:.5f}  h_speed_meas:{10:.3f}".format(curr_angle_degree, timer_0, h_speed, est_extension, h_position, encoder_measured_distance, ser.inWaiting(), lag, comp_speed, h_speed_est, h_speed_measured))
            h_position_old = h_position
            time.sleep(0.05)
            split_time_end = split_time_start
            if curr_angle_degree > 68.0:
                print("Charge port out of reach in Arm axis.")
                pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
                pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
                return False, curr_angle_degree
    print("Reached requested extension, exiting...")
    pi.write(H_DIRECTION_PIN, 0)
    pi.write(ARM_DIRECTION_PIN, 0)
    pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
    pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
    return True, curr_angle_degree

def insert_movement(start_angle_degree, arm_speed, arm_direction, distance_away):
    ser.flush()
    curr_angle_degree = start_angle_degree
    arm_speed_comp = (1 - ((1 - (arm_speed/100)) / 2)) * 100
    timer_0 = 0
    curr_angle = math.radians(curr_angle_degree)
    est_extension = math.sin(curr_angle) * arm_length
    h_start = (arm_length - (math.cos(curr_angle) * arm_length))
    h_position = 0
    h_position_old = h_position = (arm_length - (math.cos(curr_angle) * arm_length)) - h_start
    global h_speed
    h_speed = 0 #Input power amount to motor
    h_speed_est = 0 #Calculated component velocty of arm in H-direction
    h_speed_measured = 0
    split_time_end = 0
    comp_speed = 1
    line = 0
    final_extension = est_extension + distance_away + 20 #Total insertion distance
    intermediate_extension = est_extension + distance_away + 5 #Distance where it will stop doing a coordinated_extension and start angling the insertion
    angle_compensation = 1 - ((100-arm_speed)/25)/100
    encoder_measured_distance_old = 0
    encoder_measured_distance = 0
    print("Angle Compensation Factor: {0:.6f}".format(angle_compensation))
    if arm_direction == 0: #This means inserting
        pi.write(ARM_DIRECTION_PIN, 0)
        pi.write(H_DIRECTION_PIN, 0)
        start_time_0 = time.time()
        pi.set_PWM_dutycycle(ARM_SPEED_PIN, arm_speed)
        while (est_extension < intermediate_extension):
            split_time_start = time.time()
            ser.write(1)
            linestr = ser.readline().decode('utf-8').rstrip()
            try:
                line = float(linestr)
            except:
                pass
            encoder_measured_distance=(line*0.0012555)
            if arm_left_limit == True:
                pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
                pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
                print("Charge port out of reach in horizontal axis.")
                return False, curr_angle_degree
            else:
                h_speed_measured = (encoder_measured_distance_old-encoder_measured_distance)/(split_time_end-split_time_start)
                timer_0 = (time.time() - start_time_0)
                curr_angle_degree = (timer_0 * (arm_speed/100) * 2.04) + start_angle_degree + 1 #Adjust the time value here instead of angle to make work properly?
                curr_angle = math.radians(curr_angle_degree*angle_compensation)
                tangential_speed = (arm_speed/100) * 2.637053029
                h_speed_est = tangential_speed * math.sin(curr_angle)
                est_extension = math.sin(curr_angle) * arm_length
                comp_speed = (h_speed_est-h_speed_measured) * 15000
                h_position = (arm_length - (math.cos(curr_angle) * arm_length)) - h_start
                lag = h_position - encoder_measured_distance
                h_speed = int((h_speed_est*36000)) + 125000 + int(comp_speed)
                encoder_measured_distance_old = encoder_measured_distance
                if (lag > .3) and h_speed < 125000:
                    h_speed = 175000
                    pi.hardware_PWM(H_SPEED_PIN, 20000, h_speed)
                elif h_speed < 500000:
                    pi.hardware_PWM(H_SPEED_PIN, 20000, h_speed)
                print("cur_angle: {0:.1f}  est_ext: {3:.1f}cm  h_pos: {4:.1f}cm  Cur_Time: {1:.1f}s  h_speed: {2:,}  enc_dist: {5:0.1f}cm  buff_wait: {6:,}  lag: {7:.1f} comp_speed: {8:.1f}  h_speed_est: {9:.5f}  h_speed_meas:{10:.3f}".format(curr_angle_degree, timer_0, h_speed, est_extension, h_position, encoder_measured_distance, ser.inWaiting(), lag, comp_speed, h_speed_est, h_speed_measured))
                h_position_old = h_position
                time.sleep(0.05)
                split_time_end = split_time_start
                if curr_angle_degree > 68.0:
                    print("Charge port out of reach in Arm axis.")
                    pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
                    pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
                    return False, curr_angle_degree
        #pi.write(H_DIRECTION_PIN, 1)
        ser.flush()
        print("Changing Direction...")
        encoder_measured_distance_1 = encoder_measured_distance
        while (est_extension < final_extension):
            split_time_start = time.time()
            ser.write(1)
            linestr = ser.readline().decode('utf-8').rstrip()
            try:
                line = float(linestr)
            except:
                pass
            encoder_measured_distance=(line*0.0012555)
            if arm_left_limit == True:
                pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
                pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
                print("Charge port out of reach in horizontal axis.")
                return False, curr_angle_degree
            else:
                h_speed_measured = (encoder_measured_distance_old-encoder_measured_distance)/(split_time_end-split_time_start)
                timer_0 = (time.time() - start_time_0)
                curr_angle_degree = (timer_0 * (arm_speed/100) * 2.04) + start_angle_degree + 1 #Adjust the time value here instead of angle to make work properly?
                curr_angle = math.radians(curr_angle_degree*angle_compensation)
                tangential_speed = (arm_speed/100) * 2.637053029
                h_speed_est = tangential_speed * math.sin(curr_angle)
                est_extension = math.sin(curr_angle) * arm_length
                comp_speed = (h_speed_est-h_speed_measured) * 15000
                h_position = (arm_length - (math.cos(curr_angle) * arm_length)) - h_start
                lag = h_position - encoder_measured_distance
                h_speed = int((h_speed_est*6000)) + 125000 + int(comp_speed)
                target_carrier_speed = 0
                encoder_measured_distance_old = encoder_measured_distance
                if h_speed < 500000:
                    pi.hardware_PWM(H_SPEED_PIN, 20000, h_speed)
                print("cur_angle: {0:.1f}  est_ext: {3:.1f}cm  h_pos: {4:.1f}cm  Cur_Time: {1:.1f}s  h_speed: {2:,}  enc_dist: {5:0.1f}cm  buff_wait: {6:,}  lag: {7:.1f} comp_speed: {8:.1f}  h_speed_est: {9:.5f}  h_speed_meas:{10:.3f}".format(curr_angle_degree, timer_0, h_speed, est_extension, h_position, encoder_measured_distance, ser.inWaiting(), lag, comp_speed, h_speed_est, h_speed_measured))
                h_position_old = h_position
                time.sleep(0.05)
                split_time_end = split_time_start
                if curr_angle_degree > 68.0:
                    print("Charge port out of reach in Arm axis.")
                    pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
                    pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
                    return False, curr_angle_degree

        print("Reached requested extension, exiting...")
        pi.write(H_DIRECTION_PIN, 0)
        pi.write(ARM_DIRECTION_PIN, 0)
        pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
        pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
        return True, curr_angle_degree
    else: #This means extracting
        pi.write(ARM_DIRECTION_PIN, 1)
        pi.write(H_DIRECTION_PIN, 1)
    start_time_0 = time.time()
    pi.set_PWM_dutycycle(ARM_SPEED_PIN, arm_speed)



def main():

    requested_extension = 75
    start_angle_degree = 5
    arm_speed = 100
    arm_direction = 0
    distance_away = 20 #cm
    global ser
    ser = serial.Serial('/dev/ttyUSB0', 74880, timeout=.1)
    time.sleep(2)
    #arm_status = coordinated_extension(requested_extension, start_angle_degree, arm_speed, arm_direction)
    arm_status = insert_movement(start_angle_degree, arm_speed, arm_direction, distance_away)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Process stopped by User")
        pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
        pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
        pi.set_servo_pulsewidth(SERVO, 0)
        pi.stop()
    except Exception:
        print("Process Failed. See below.")
        pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
        pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
        pi.set_servo_pulsewidth(SERVO, 0)
        pi.stop()
        traceback.print_exc()
