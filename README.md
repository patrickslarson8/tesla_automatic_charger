# tesla_charger_arm
A machine learning model for plugging in a Tesla Model 3

A video of it in action [here](https://www.youtube.comwatchv=octvXMaTG44&t)

Uses the Tensorflow object detection API found [here]
(https://github.comtensorflowmodelstreemasterresearchobject_detection)

My script uses a modified version of detect.py from this [repository]
(https://github.comtensorflowexamplestreemasterliteexamplesobject_detectionraspberry_pi)

##Directory Scructure
###Data Labelling
- Includes helper scripts and example .csv files of labels
- Pictures labeled using [labelimg:](https://github.com/heartexlabs/labelImg)
- Labels converted from xml to csv with inclded scripts.
- Scripts are from [Dat Tran's Raccoon Dataset](https://github.com/datitran/raccoon_dataset)
- .csv then converted to tfrecord files for use in training

###Images
- Screenshots of tensorboard output to show model performance
- The actual images used in training with their labels

###Model Training
- Includes annotations that are used during inferencing
- start_training.txt is a list of commands used to train this model

###Raspberry Pi Files