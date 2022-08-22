## This program is designed to automatically plug in a tesla charging handle
## using a Raspberry pi 4, camera, ultrasonic sensor, servo driver and two DC
## motor drivers.
##############################################################################
##pseudo code:                                                    (state #) ##
##if car detection yes                      v1                    (state 1) ##
##check using tesla api                     v1                    (state 1) ##
##       turn on light if needed            xx                    (state 1) ##
##       start webstreaming for diagnostics xx                    (state 1) ##
##    find charge port door                 v0.5                  (state 2) ##
##    (get closer? Not sure if necessary)   v1                    (state 3) ##
##    open charge door                      v1                    (state 4) ##
##    find charge port                      v1                    (state 4) ##
##    get really close                      v1                    (state 5) ##
##    pull charge handle down               v0.5                  (state 6) ##
##    insert                                v0.5                  (state 7) ##
##    verify charge status                  v0.8                  (state 8) ##
##    wait charge complete                  v1                    (state 8) ##
##        (determine when to pull if charge complete vs schedule departure) ##
##        (also determine method to request early pull-out)       (state 8) ##
##    extract charge handle                 v0.5                  (state 9) ##
##    stow                                  v1                    (state 9) ##
##    wait for car to leave                 v1                    (state 10)##
##    reset to waiting for car detection    v1                    (state 10)##
##############################################################################

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
logo_centering_speed = 150000
logo_centering_step_size = 14
logo_center_location = 0.78
logo_center_precision = 0.015
extract_distance = 30
insert_step_size = 0.7 #time that arm will move before taking another distance reading
insert_buffer = 2 #amount in cm that if close enough will stop pushing into the car
arm_length = 77.8

pi = pigpio.pi()

##pinmap
ARM_DIRECTION_PIN = 11
ARM_SPEED_PIN = 9
H_DIRECTION_PIN = 10
H_SPEED_PIN = 13
SERVO = 26
L_LIMIT = 20
R_LIMIT = 21

## Prep the battlefield
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

#distance sensor
def get_distance():
    raw_dist = us100.distance
    print("Distance: {0:.2f}".format(raw_dist))
    return raw_dist

## Movement helper function
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
            print("Requested extension too far, exiting.")
            return False, curr_angle_degree
    else:
        pi.write(ARM_DIRECTION_PIN, 1)
        pi.write(H_DIRECTION_PIN, 1)
    start_time_0 = time.time()
    pi.set_PWM_dutycycle(ARM_SPEED_PIN, arm_speed)
    while (est_extension < final_extension):
        split_time_start = time.time()
        ser.write('1'.encode('utf-8'))
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
            #print("cur_angle: {0:.1f}  est_ext: {3:.1f}cm  h_pos: {4:.1f}cm  Cur_Time: {1:.1f}s  h_speed: {2:,}  enc_dist: {5:0.1f}cm  buff_wait: {6:,}  lag: {7:.1f} comp_speed: {8:.1f}  h_speed_est: {9:.5f}  h_speed_meas:{10:.3f}".format(curr_angle_degree, timer_0, h_speed, est_extension, h_position, encoder_measured_distance, ser.inWaiting(), lag, comp_speed, h_speed_est, h_speed_measured))
            h_position_old = h_position
            time.sleep(0.05)
            split_time_end = split_time_start
            if curr_angle_degree > 68.0:
                print("Charge port out of reach in Arm axis.")
                pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
                pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
                return False, curr_angle_degree
    print("Reached requested extension, exiting.")
    pi.write(H_DIRECTION_PIN, 0)
    pi.write(ARM_DIRECTION_PIN, 0)
    pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
    pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
    return True, curr_angle_degree

def insert_movement(start_angle_degree, arm_speed, arm_direction, distance_away):
    ser.write('2'.encode('utf-8'))
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
    final_extension = est_extension + distance_away - 1.5 #Total insertion distance
    intermediate_extension = est_extension + distance_away - 19.8 #Distance where it will stop doing a coordinated_extension and start angling the insertion
    print("est_extension: {0:.2f}   intermediate_extension: {1:.2f}  final_extension: {2:.2f}  dist: {3:.2f}".format(est_extension, intermediate_extension, final_extension, distance_away))
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
            ser.write('1'.encode('utf-8'))
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
            ser.write('1'.encode('utf-8'))
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
                comp_speed = (h_speed_est-h_speed_measured) * 200
                h_position = (arm_length - (math.cos(curr_angle) * arm_length)) - h_start
                lag = h_position - encoder_measured_distance
                h_speed = int((h_speed_est*2000)) + 90000 + int(comp_speed)
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
        print("Should be plugged in. Exiting.")
        pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
        pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
        return True, curr_angle_degree

## Tesla API auth helper functions
def get_passcode():
    #Need to adjust so that it alerts if it needs a passcode
    return raw_input('Passcode: ')

def select_factor(factors):
    print('-'*80)
    print('ID Name')
    for i, factor in enumerate(factors):
        print('{:2} {}'.format(i, factor['name']))
    print('-'*80)
    idx = int(raw_input('Select factor: '))
    print('-'*80)
    return factors[idx]

def update_tesla_status(vehicle_index):
    #Need to update so password isn't in plaintext here
    try:
        with teslapy.Tesla('youremail@gmail.com', 'yourpassword', get_passcode, select_factor) as tesla:
            global drive_state
            global charge_state
            global lat
            global lon
            global vehicles
            tesla.fetch_token()
            vehicles = tesla.vehicle_list()
            print("Waiting for LSP to wake up...")
            vehicles[vehicle_index].sync_wake_up()
            print("Awoken!")
            data = vehicles[0].get_vehicle_data()
            #print(data)
            drive_state = data["drive_state"]
            charge_state = data["charge_state"]
            lat = drive_state["latitude"]
            lon = drive_state["longitude"]
            #print(charge_state["battery_level"])
            #print(charge_state["timestamp"])
            return 0
    except:
            return 1

def is_LSP_home():
    # Need to add HTTP timeout handling at very least
    counter = 2
    while counter > 0:
        try:
            if update_tesla_status(0) == 0:
                if (((lat > 31.3522) and (lat < 31.3524)) and ((lon > -85.8594) and (lon < -8.8592))):
                    print("LSP is Home!")
                    return int(0)
                else:
                    print("LSP is away :(")
                    return 1
        except:
            counter = counter - 1
            time.sleep(5)
    return 2
    #print("simulating LSP is home...")
    #return True

def open_charge_port():
    counter = 2
    while counter > 0:
        try:
            with teslapy.Tesla('youremail@gmail.com', 'yourpassword', get_passcode, select_factor) as tesla:
                tesla.fetch_token()
                vehicles = tesla.vehicle_list()
                vehicles[0].command('CHARGE_PORT_DOOR_OPEN')
                print("Opening charge port door...")
                time.sleep(2)
                return 0
        except:
            counter = counter - 1
            time.sleep(5)
    return 1

def is_charge_port_open():
    counter = 2
    while counter > 0:
        try:
            update_tesla_status(0)
            counter = 0
        except:
            counter = counter - 1
    if charge_state["charge_port_door_open"] == "true":
        print("Charge port is open")
        return 0
    else:
        print("Charge port not open.")
        return 1


def is_LSP_plugged_in():
    counter = 2
    while counter > 0:
        try:
            update_tesla_status(0)
            counter = 0
        except:
            counter = counter - 1
        if charge_state["charging_state"] != "Disconnected":
            return 0
        else:
            return 1

def is_LSP_latched():
    counter = 2
    while counter > 0:
        try:
            update_tesla_status(0)
            counter = 0
        except:
            counter = counter - 1
        if charge_state["charge_port_latch"] == "Engaged":
            return 0
        else:
            return 1

def get_charge_end_time():
    #Need to add ability for scheduled charge start time
    counter = 2
    while counter > 0:
        try:
            update_tesla_status(0)
            counter = 0
        except:
            counter = counter - 1
        if "scheduled_departure_time" in charge_state:
            minutes_to_full_charge = charge_state["scheduled_departure_time"] - charge_state["timestamp"]
            seconds_to_full_charge = int(minutes_to_full_charge * 60)
            return seconds_to_full_charge
        elif charge_state["charge_enable_request"] == "true":
            return int(charge_state["minutes_to_full_charge"]) * 60
        else:
            return 1

## Inference helper functions
def get_center(xmax,xmin):
    center = ((xmax-xmin)/2)+xmin
    return center

def load_labels(path):
  """Loads the labels file. Supports files with or without index numbers."""
  with open(path, 'r', encoding='utf-8') as f:
    lines = f.readlines()
    labels = {}
    for row_number, content in enumerate(lines):
      pair = re.split(r'[:\s]+', content.strip(), maxsplit=1)
      if len(pair) == 2 and pair[0].strip().isdigit():
        labels[int(pair[0])] = pair[1].strip()
      else:
        labels[row_number] = pair[0].strip()
  return labels


def set_input_tensor(interpreter, image):
  """Sets the input tensor."""
  tensor_index = interpreter.get_input_details()[0]['index']
  input_tensor = interpreter.tensor(tensor_index)()[0]
  input_tensor[:, :] = image


def get_output_tensor(interpreter, index):
  """Returns the output tensor at the given index."""
  output_details = interpreter.get_output_details()[index]
  tensor = np.squeeze(interpreter.get_tensor(output_details['index']))
  return tensor


def detect_objects(interpreter, image, threshold):
  """Returns a list of detection results, each a dictionary of object info."""
  print("Detecting objects with a score greater than {}".format(threshold))
  set_input_tensor(interpreter, image)
  interpreter.invoke()

  # Get all output details
  boxes = get_output_tensor(interpreter, 0)
  classes = get_output_tensor(interpreter, 1)
  scores = get_output_tensor(interpreter, 2)
  count = int(get_output_tensor(interpreter, 3))

  results = []
  for i in range(count):
    if scores[i] >= threshold:
      result = {
          'bounding_box': boxes[i],
          'class_id': classes[i],
          'score': scores[i]
      }
      results.append(result)
  print(results)
  return results


def annotate_objects(annotator, results, labels):
  """Draws the bounding box and label for each object in the results."""
  for obj in results:
    # Convert the bounding box figures from relative coordinates
    # to absolute coordinates based on the original resolution
    ymin, xmin, ymax, xmax = obj['bounding_box']
    xmin = int(xmin * CAMERA_WIDTH)
    xmax = int(xmax * CAMERA_WIDTH)
    ymin = int(ymin * CAMERA_HEIGHT)
    ymax = int(ymax * CAMERA_HEIGHT)

    # Overlay the box, label, and score on the camera preview
    annotator.bounding_box([xmin, ymin, xmax, ymax])
    annotator.text([xmin, ymin],
                   '%s\n%.2f' % (labels[obj['class_id']], obj['score']))

def get_results(input_width, input_height, interpreter):
    counter = 2
    while counter > 0:
        try:
            with picamera.PiCamera(resolution=(CAMERA_WIDTH, CAMERA_HEIGHT), framerate=30) as camera:
                camera.rotation = 270
                #camera.start_preview() #Turn into html stream to make headless
                print("Initialized camera.")
                stream = io.BytesIO()
                for _ in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
                    stream.truncate() # Added as per RPI Camera Docs example
                    stream.seek(0)
                    image = Image.open(stream).convert('RGB').resize((input_width, input_height), Image.ANTIALIAS)
                    #print(image)
                    start_time = time.monotonic()
                    results = detect_objects(interpreter, image, score_threshold)
                    elapsed_ms = (time.monotonic() - start_time) * 1000
                    stream.seek(0)
                    stream.truncate()
                    return results, elapsed_ms
        except:
            time.sleep(1)
            counter = counter - 1
    print("Unable to get video. Tried twice, exiting.")
    quit()


def main():
    ## Initialize, to include verifing current state
    state = 0 # for search algorithm
    global arm_left_limit
    global arm_right_limit
    arm_left_limit = True
    arm_right_limit = True
    global ser
    ser = serial.Serial('/dev/ttyUSB0', 74880, timeout=.1)
    uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=3000)
    # Create a US-100 module instance.
    global us100
    us100 = adafruit_us100.US100(uart)
    print("Initialized distance sensor")
    labels = load_labels(label_path)
    print("Sucessfully loaded labels.")
    interpreter = Interpreter(model_path)
    interpreter.allocate_tensors()
    print("Sucessfully allocated resources for inferencing.")
    client1= paho.Client("control1")                           #create client object
    client1.on_publish = on_publish                          #assign function to callback
    client1.connect(broker,port)                                 #establish connection
    print("Connected to zigbee light")
    _, input_height, input_width, _ = interpreter.get_input_details()[0]['shape']
    print("Entering Main loop.")
    while True:
        if state == 0:
            print("Internal state is 0.")
            # Make this a section to re-initialize everything if there is a power outage
            if pi.read(L_LIMIT) == 1:
                arm_left_limit = False
            if pi.read(R_LIMIT) == 1:
                arm_right_limit = False
            # Need to add check if arm was plugged in before it just willy-nilly resets
            print("Resetting arm position...")
            while arm_right_limit == False:
                pi.write(H_DIRECTION_PIN, 1)
                pi.hardware_PWM(H_SPEED_PIN, 20000, 250000)
                if pi.read(R_LIMIT) == 0:
                    arm_right_limit = True
            pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
            pi.write(ARM_DIRECTION_PIN, 1)
            pi.set_PWM_dutycycle(ARM_SPEED_PIN, 100)
            #time.sleep(35) #when not using the rest function re-add this
            print("Arm position reset.")
            pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
            arm_status = (True, 0)
            dist = 500
            state = 1
        elif state == 1:
            print("Internal state is 1.")
            #wait for car to arrive
            #Not complete needs to be developed
            #   add logic for if car is home but fully charged
            print("waiting for car to arrive...")
            ret= client1.publish("zigbee2mqtt/arm_plugger_light/set",'{"state":"OFF"}')
            if dist < detect_distance:
                print("Detected something close!")
                if is_LSP_home() == 0:
                    state = 2
                else:
                    print("False alarm.")
                    dist = get_distance()
                    state = 1
            else:
                #time.sleep(idle_sleep_time)
                dist = get_distance()
        elif state == 2:
            print("Internal state is 2.")
            #find charge port door
            ret= client1.publish("zigbee2mqtt/arm_plugger_light/set",'{"state":"ON"}')
            time.sleep(1)
            results, elapsed_ms = get_results(input_height, input_width, interpreter)
            if arm_left_limit == True:
                pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
                state = 0
            elif len(results) > 0:
                if (results[0]['class_id'] == 1.0) or (results[0]['class_id'] == 0.0):#top result is charge port or logo:
                    state = 4
                    print("Port or logo detected")
                elif (results[0]['class_id'] == 2.0): # top result is reflector
                    state = 3
                    print("Reflector Detected")
            else:
                #Add ability to gauge how far you've gone
                pi.write(H_DIRECTION_PIN, 0)
                pi.hardware_PWM(H_SPEED_PIN, 20000, search_move_speed)
                time.sleep(search_step_size)
                pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
        elif state == 3:
            print("Internal state is 3.")
            results, elapsed_ms = get_results(input_height, input_width, interpreter)
            if len(results) == 0:
                state = 2
                print("Found something but it went away")
            elif (results[0]['class_id'] == 1.0) or (results[0]['class_id'] == 0.0):#top result is charge port or logo:
                state = 4
                print("Found next thing, changing state")
            else:
                ymin, xmin, ymax, xmax = results[0]['bounding_box']
                # Below is currently pseudo code
                print("Xmax: {0:,} Xmin: {1:,} Xlocation: {2:,}".format(xmax,xmin,get_center(xmax,xmin)))
                if get_center(xmax,xmin) > 0.8: #too far one direction
                    print("Xlocation too far forward, moving back")
                    pi.write(H_DIRECTION_PIN, 0)
                    pi.hardware_PWM(H_SPEED_PIN, 20000, centering_speed)
                    time.sleep((get_center(xmax,xmin)-0.65)*centering_step_size)
                    pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
                elif get_center(xmax,xmin) < 0.7: #too far other direction
                    print("Xlocation too far back, moving forward")
                    pi.write(H_DIRECTION_PIN, 1)
                    pi.hardware_PWM(H_SPEED_PIN, 20000, centering_speed)
                    time.sleep(abs((get_center(xmax,xmin)-0.65))*centering_step_size)
                    pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
                else:
                    print("Xlocation of reflector relatively centered, moving on")
                    state = 4
        elif state == 4:
            # find charge port
            ret= client1.publish("zigbee2mqtt/arm_plugger_light/set",'{"state":"OFF"}')
            if is_charge_port_open() == 1:
                try:
                    open_charge_port()
                except:
                    pass
            dist = get_distance()
            requested_extension = dist - port_detect_distance
            if requested_extension > 0:
                arm_status = coordinated_extension(requested_extension, 0, 100, 0)
                time.sleep(1)
            elif requested_extension < 0:
                arm_status = coordinated_extension(abs(requested_extension), 0, 100, 1)
                time.sleep(1)
            results, elapsed_ms = get_results(input_height, input_width, interpreter)
            if len(results) > 0:
                if (results[0]['class_id'] == 1.0): # Top result is port
                    ymin, xmin, ymax, xmax = results[0]['bounding_box']
                    print("Internal state is 4. Detected Port. Measured X-location: {0:.2f}".format(get_center(xmax,xmin)))
                    #and move to center with port x_offset
                    state = 5
                elif (results[0]['class_id'] == 0.0): # Top result is logo
                    ymin, xmin, ymax, xmax = results[0]['bounding_box']
                    print("Internal state is 4. Detected logo. Measured X-location: {0:.2f}".format(get_center(xmax,xmin)))
                    #and move to center with logo x_offset
                    state = 5
                else:
                    print("Internal State is 4. Reflector Detected. Waiting on charge port or logo to show up.")
            else:
                print("No luck, continue looking...")
                state = 2
        elif state == 5:
            #get really close
            dist = get_distance()
            requested_extension = dist - really_close_distance
            print("Internal state is 5. Getting really close. Requested extension: {0:.2f}".format(requested_extension))
            arm_status = coordinated_extension(requested_extension, arm_status[1], 80, 0)
            state = 6
        elif state == 6:
            #rotate
            print("Internal state is 6.")
            dist = get_distance()
            if dist > rotate_clearance_distance:
                for x in range(720):
                    servo_position = 800 + x
                    pi.set_servo_pulsewidth(SERVO, servo_position)
                    time.sleep(0.007)
                state = 7
            else:
                state = 5
        elif state == 7:
            #insert
            print("Internal state is 7.")
            #Turn off light to find logo
            dist = get_distance()
            if dist < really_close_distance:
                requested_extension - dist - really_close_distance
                arm_status = coordinated_extension(requested_extension, arm_status[1], 50, 0)
            results, elapsed_ms = get_results(input_height, input_width, interpreter)
            match = next((item for item in results if item['class_id']==0.0), None)
            print(match)
            if match != None:
                ymin, xmin, ymax, xmax = match['bounding_box']
                if get_center(xmax,xmin) > (logo_center_location + logo_center_precision): #too far one direction
                    print("Internal state is 7. Measured X-location: {0:.2f}".format(get_center(xmax,xmin)))
                    pi.write(H_DIRECTION_PIN, 0)
                    pi.hardware_PWM(H_SPEED_PIN, 20000, logo_centering_speed)
                    time.sleep((get_center(xmax,xmin)-logo_center_location)*logo_centering_step_size)
                    pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
                elif get_center(xmax,xmin) < (logo_center_location - logo_center_precision): #too far other direction
                    print("Internal state is 7. Measured X-location: {0:.2f}".format(get_center(xmax,xmin)))
                    pi.write(H_DIRECTION_PIN, 1)
                    pi.hardware_PWM(H_SPEED_PIN, 20000, logo_centering_speed)
                    time.sleep(abs((get_center(xmax,xmin)-logo_center_location))*logo_centering_step_size)
                    pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
                else:
                    print("Xlocation of logo centered, moving on")
                    dists = []
                    for _ in range(3):
                        time.sleep(0.7)
                        dists.append(get_distance())
                    dist = statistics.median(dists)
                    requested_extension = dist - insert_distance
                    #def insert_movement(arm_status, move_amount, arm_speed, arm_direction):
                    arm_status = insert_movement(arm_status[1], insert_speed, 0, dist)
                    state = 8
            else:
                print("Logo not found but it should be right in front of me.")
                state = 4
        elif state == 8:
            #verify charge status, wait for charge, determine when charged
            print("Internal state is 8.")
            time.sleep(5)
            if is_LSP_plugged_in() == 0:
                print("Successfully plugged in!")
                charge_end_time = get_charge_end_time()
                charge_sleep_time = (charge_end_time - time.time())
                if charge_sleep_time < 6000:
                    if is_LSP_charging() == False:
                        state = 9
                    else:
                        time.sleep(120)
                else:
                    time.sleep(charge_sleep_time - 6000)
            else:
                print("Failed to plug in. Trying a little more.")
                pi.set_PWM_dutycycle(ARM_SPEED_PIN, 50)
                time.sleep(insert_step_size)
                if is_LSP_plugged_in() == 0:
                    break
                else:
                    state = 4
        elif state == 9:
            #extract, stow
            print("Internal state is 9. Pulling out to reset.")
            if is_LSP_latched() == 0:
                open_charge_port()
                time.sleep(1)
            requested_extension = extract_distance
            arm_status = insert_movement(arm_status[1],requested_extension,insert_speed,1)
            print("Resetting arm position...")
            while arm_right_limit == False:
                pi.write(H_DIRECTION_PIN, 1)
                pi.hardware_PWM(H_SPEED_PIN, 20000, 250000)
            pi.hardware_PWM(H_SPEED_PIN, 20000, 0)
            pi.write(ARM_DIRECTION_PIN, 1)
            pi.set_PWM_dutycycle(ARM_SPEED_PIN, 100)
            time.sleep(60)
            pi.set_PWM_dutycycle(ARM_SPEED_PIN, 0)
            print("Arm reset.")
            state = 10
        elif state == 10:
            #wait for car to leave, then reset
            #This presumes that when I leave I will be gone for at least 10 minutes
            print("Internal state is 10.")
            if get_distance() < detect_distance:
                time.sleep(600)
            elif get_distance() > detect_distance:
                state = 1

## Run when called as main
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
