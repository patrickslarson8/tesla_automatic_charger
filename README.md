# tesla_charger_arm
A machine learning model for plugging in a Tesla Model 3

A video of it in action [here](https://www.youtube.comwatchv=octvXMaTG44&t)

Uses the Tensorflow object detection API found [here](https://github.comtensorflowmodelstreemasterresearchobject_detection)

My script uses a modified version of detect.py from this [repository](https://github.comtensorflowexamplestreemasterliteexamplesobject_detectionraspberry_pi)

Picture of training metrics from Tensorboard:
![Network Performance Metrics](/Images/Screenshots/tensorboard_screenshot_1.jpg)

Interacts with the vehicle using the [TeslaPy package](https://github.com/tdorssers/TeslaPy)

# Hardware
Raspberry Pi 4
- [RPi Camera V2](https://www.raspberrypi.com/products/camera-module-v2/)

Horizontal Motor
- [Pololu Gearmotor](https://www.pololu.com/product/4841)
- [Pololu Arduino Motor Controller](https://www.pololu.com/product/2520)

Plugger end control
- [Generic servo](https://www.amazon.com/gp/product/B07KTSCN4J/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)

Zigbee
- [Generic Zigbee Controller](https://www.amazon.com/gp/product/B07P5LY7Z6/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1)

Distance Sensor
- [Adafruit US-100](https://www.adafruit.com/product/4019#tutorials)

## Directory Scructure
### Data Labelling
- Includes helper scripts and example .csv files of labels
- Pictures labeled using [labelimg:](https://github.com/heartexlabs/labelImg)
- Labels converted from xml to csv with included scripts.
- Scripts are from [Dat Tran's Raccoon Dataset](https://github.com/datitran/raccoon_dataset)
- .csv then converted to tfrecord files for use in training

### Images
- Screenshots of tensorboard output to show model performance
- The actual images used in training with their labels

### Model Training
- Includes annotations that are used during inferencing
- start_training.txt is a list of commands used to train this model

### Raspberry Pi Files
- arm_plugger.py is the main script which is run to detect and
plug in the arm
- Motor Control contains scripts that were used to fine tune
mechanical movement
- Component testing contains scripts used to verify component
operation e.g. the Zigbee light, communication with arduino,
and servo operation.
- detect.tflite is the Tensorflow Lite model used to inference
the pictures taken by the RPi in operation.
