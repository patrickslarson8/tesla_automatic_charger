# tesla_charger_arm
A machine learning model for plugging in a Tesla Model 3

A video of it in action here
httpswww.youtube.comwatchv=octvXMaTG44&t

Uses the Tensorflow object detection API found here
httpsgithub.comtensorflowmodelstreemasterresearchobject_detection

My script uses a modified version of 
detect.py from this repository
httpsgithub.comtensorflowexamplestreemasterliteexamplesobject_detectionraspberry_pi

##Directory Scructure
###Testing Scripts

###Tensorflow Model
- detect_picamera.py used to visualize the results of the
detection algorithm by displaying the camera feed live.
Must use the physical hdmi port on the Raspberry pi.

###Annotations
  - label_lite.txt used by scripts to translate activated
neurons to the appropriate object category

  - label_map.pbtxt used by the tensorflow model to translate
detections to the correct category label

  - test.record
  - train.record
  
###Model
  - 22774.tflite is a tflite model converted from a checkpoint
  of the SSD resnet50 640x640 pre-trained model available here
  httpsgithub.comtensorflowmodelsblobmasterresearchobject_detectiong3doctf2_detection_zoo.md
  
  - detect.tflite is the same as 22774.tflite but renamed
  to work with existing scripts
