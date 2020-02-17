######## Webcam Object Detection Using Tensorflow-trained Classifier #########
#
# Author: Evan Juras
# Date: 1/20/18
# Description: 
# This program uses a TensorFlow-trained classifier to perform object detection.
# It loads the classifier uses it to perform object detection on a webcam feed.
# It draws boxes and scores around the objects of interest in each frame from
# the webcam.

## Some of the code is copied from Google's example at
## https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb

## and some is copied from Dat Tran's example at
## https://github.com/datitran/object_detector_app/blob/master/object_detection_app.py

## but I changed it to make it more understandable to me.


# Import packages
import os
import cv2
import numpy as np
import tensorflow as tf
import sys
import math
import numpy as np
import threading,sys,time
import DobotDllType as dType
import imutils

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")

# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util
#from utils.visualization_utils import draw_bounding_box_on_image as draw1

#global xco,yco
cX = None


#############################################################################################################
#############################      DOBOT CODE FOR CAM  #############################################################
#############################################################################################################
def dohome():
	CON_STR = {
		dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
		dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
		dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

	#Load Dll
	api = dType.load()
		
	#Connect Dobot
	state = dType.ConnectDobot(api, "", 115200)[0]
	print("Connect status:",CON_STR[state])

	if (state == dType.DobotConnect.DobotConnect_NoError):

		#Clean Command Queued
		dType.SetQueuedCmdClear(api)
			
		#Async Home
		lastIndex = dType.SetHOMECmd(api, temp = 0, isQueued = 1)[0]
		lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 200, 0, 170, 0, isQueued = 1)[0]

		#Start to Execute Command Queued
		dType.SetQueuedCmdStartExec(api)

		#Wait for Executing Last Command 
		while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
			dType.dSleep(100)

		#Stop to Execute Command Queued
		dType.SetQueuedCmdStopExec(api)

	#Disconnect Dobot
	dType.DisconnectDobot(api)
	
	
#############################################################################################################
#############################      DOBOT CODE FOR CAM  #############################################################
#############################################################################################################
def dohome1():
	CON_STR = {
		dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
		dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
		dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

	#Load Dll
	api = dType.load()
		
	#Connect Dobot
	state = dType.ConnectDobot(api, "", 115200)[0]
	print("Connect status:",CON_STR[state])

	if (state == dType.DobotConnect.DobotConnect_NoError):

		#Clean Command Queued
		dType.SetQueuedCmdClear(api)
			
		#Async Home
		#lastIndex = dType.SetHOMECmd(api, temp = 0, isQueued = 1)[0]
		lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 250, 0, 50, 0, isQueued = 1)[0]

		#Start to Execute Command Queued
		dType.SetQueuedCmdStartExec(api)

		#Wait for Executing Last Command 
		while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
			dType.dSleep(100)

		#Stop to Execute Command Queued
		dType.SetQueuedCmdStopExec(api)

	#Disconnect Dobot
	dType.DisconnectDobot(api)


#############################################################################################################
#############################      DOBOT CODE   #############################################################
#############################################################################################################
def dobot_exe_bolt(dX,dY,dZ,dR,j):
	CON_STR = {
		dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
		dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
		dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

	#Load Dll
	api = dType.load()
	

	#Connect Dobot
	state = dType.ConnectDobot(api, "", 115200)[0]
	print("Connect status:",CON_STR[state])

	if (state == dType.DobotConnect.DobotConnect_NoError):

		#Clean Command Queued
		dType.SetQueuedCmdClear(api)
		
		#Async Home
		#dType.SetHOMECmd(api, temp = 0, isQueued = 1)
		dType.SetPTPJumpParams(api,30,140)
		lastIndex = dType.SetIODO(api, 2, 1, 1)[0]
		lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPJUMPXYZMode, dX, dY, dZ, 0, isQueued = 1)[0]
		#lastIndex = dType.SetIODO(api, 2, 1, 1)[0]
		lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPJUMPXYZMode, 81.4, 198.45, -52.7+j, dR, isQueued = 1)[0]
		lastIndex = dType.SetIODO(api, 2, 0, 1)[0]
		
		#Start to Execute Command Queued
		dType.SetQueuedCmdStartExec(api)

		#Wait for Executing Last Command 
		while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
			dType.dSleep(100)

		#Stop to Execute Command Queued
		dType.SetQueuedCmdStopExec(api)

	#Disconnect Dobot
	dType.DisconnectDobot(api)

#############################################################################################################
#############################      DOBOT CODE   #############################################################
#############################################################################################################
def dobot_exe_nut(dX,dY,dZ,dR,j):
	CON_STR = {
		dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
		dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
		dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

	#Load Dll
	api = dType.load()
	

	#Connect Dobot
	state = dType.ConnectDobot(api, "", 115200)[0]
	print("Connect status:",CON_STR[state])

	if (state == dType.DobotConnect.DobotConnect_NoError):

		#Clean Command Queued
		dType.SetQueuedCmdClear(api)
		
		#Async Home
		#dType.SetHOMECmd(api, temp = 0, isQueued = 1)
		dType.SetPTPJumpParams(api,30,140)
		lastIndex = dType.SetIODO(api, 2, 1, 1)[0]
		lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPJUMPXYZMode, dX, dY, dZ, 0, isQueued = 1)[0]
		#lastIndex = dType.SetIODO(api, 2, 1, 1)[0]
		lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPJUMPXYZMode, 180, -198.45, -52.7+j, dR, isQueued = 1)[0]
		lastIndex = dType.SetIODO(api, 2, 0, 1)[0]
		
		#Start to Execute Command Queued
		dType.SetQueuedCmdStartExec(api)

		#Wait for Executing Last Command 
		while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
			dType.dSleep(100)

		#Stop to Execute Command Queued
		dType.SetQueuedCmdStopExec(api)

	#Disconnect Dobot
	dType.DisconnectDobot(api)


####################################################################################
########### OBJECT DETECTION #######################################################
####################################################################################

# Name of the directory containing the object detection module we're using
MODEL_NAME = 'inference_graph'

# Grab path to current working directory
CWD_PATH = os.getcwd()

# Path to frozen detection graph .pb file, which contains the model that is used
# for object detection.
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'frozen_inference_graph.pb')

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH,'training','labelmap.pbtxt')

# Number of classes the object detector can identify
NUM_CLASSES = 3

## Load the label map.
# Label maps map indices to category names, so that when our convolution
# network predicts `5`, we know that this corresponds to `king`.
# Here we use internal utility functions, but anything that returns a
# dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)
#print(categories)

# Load the Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.Session(graph=detection_graph)


# Define input and output tensors (i.e. data) for the object detection classifier

# Input tensor is the image
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

# Output tensors are the detection boxes, scores, and classes
# Each box represents a part of the image where a particular object was detected
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

# Each score represents level of confidence for each of the objects.
# The score is shown on the result image, together with the class label.
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

# Number of objects detected
num_detections = detection_graph.get_tensor_by_name('num_detections:0')

dohome()
# Initialize webcam feed
video = cv2.VideoCapture(cv2.CAP_DSHOW)
ret = video.set(3,1280)
ret = video.set(4,720)
#ret = video.set(15, 10.0)#exposure
#ret = video.set(10, 0.0)#brighness
#ret = video.set(11, 25.0)#contrast
#ret = video.set(13, 13.0) # 13.0 hue
i = 0
while i < 2:
	input('Press Enter to capture') #python3
	#raw_input('Press Enter to capture') #python2
	ret, frame = video.read()
    #cv2.imwrite('opencv'+str(i)+'.png', image)
	i += 1
del(video)
#cv2.imshow("Image", frame)
#cv2.waitKey(0)	

resized = imutils.resize(frame, width=300)
ratio = frame.shape[0] / float(resized.shape[0])
# Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
# i.e. a single-column array, where each item in the column has the pixel RGB value
#ret, frame = video.read()
frame_expanded = np.expand_dims(resized, axis=0)

# Perform the actual detection by running the model with the image as input
(boxes, scores, classes, num) = sess.run(
        [detection_boxes, detection_scores, detection_classes, num_detections],
        feed_dict={image_tensor: frame_expanded})
#print(boxes, scores, classes, num)
    # Draw the results of the detection (aka 'visulaize the results')
vis_util.visualize_boxes_and_labels_on_image_array(
        frame,
        np.squeeze(boxes),
        np.squeeze(classes).astype(np.int32),
        np.squeeze(scores),
        category_index,
        use_normalized_coordinates=True,
        line_thickness=8,
        min_score_thresh=0.60)
#print(vis_util.lm)

#new readings with magnet	
x1=145#121 #149 152.7 121.5
y1=-85 #-50 #-49  50.8 -41
r1=0
dohome1()
'''
#print(vis_util.cX)
dXa= x1+(260-(vis_util.cX*0.21))
dYa= y1+((vis_util.cY*0.21))
dZa= -42.7
		
#print(dXa,dYa,dZa)
#print(vis_util.im_width,vis_util.im_height)
#dobot_exe(dXa,dYa,dZa,0,0)
'''
# All the results have been drawn on the frame, so it's time to display it.
cv2.imshow('Object detector', frame)
cv2.waitKey(0)

for ir in vis_util.lm:
	for im in ir:
		if im == "['BOLT']":
			cX= ir[1]
			cY= ir[2]
			print(cX,cY)
			dXa= x1+(210-(cX*0.19)) #x1+(260-(cX*0.21))
			dYa= y1+((cY*0.19))#y1+((cY*0.20))
			dZa= -63#-90.7
			print(dXa,dYa,dZa)
			dobot_exe_bolt(dXa,dYa,dZa,0,0)
		elif im == "['NUT']":
			cX= ir[1]
			cY= ir[2]
			dXa= x1+(210-(cX*0.19))#x1+(260-(cX*0.21))
			dYa= y1+((cY*0.19))#y1+((cY*0.20))
			dZa= -63#-95.7
			print(dXa,dYa,dZa)
			dobot_exe_nut(dXa,dYa,dZa,0,0)
		else:
			print()

'''		
# Press 'q' to quit

	if cv2.waitKey(1) == ord('q'):
		break

# Clean up
video.release()
cv2.destroyAllWindows()
'''

'''worked with new stainless steel electromagnet on dobot magecian v2 on 16th july 2019
for ir in vis_util.lm:
	for im in ir:
		if im == "['BOLT']":
			cX= ir[1]
			cY= ir[2]
			print(cX,cY)
			dXa= x1+(210-(cX*0.19)) #x1+(260-(cX*0.21))
			dYa= y1+((cY*0.19))#y1+((cY*0.20))
			dZa= -63#-90.7
			print(dXa,dYa,dZa)
			dobot_exe_bolt(dXa,dYa,dZa,0,0)
		elif im == "['NUT']":
			cX= ir[1]
			cY= ir[2]
			dXa= x1+(210-(cX*0.19))#x1+(260-(cX*0.21))
			dYa= y1+((cY*0.19))#y1+((cY*0.20))
			dZa= -63#-95.7
			print(dXa,dYa,dZa)
			dobot_exe_nut(dXa,dYa,dZa,0,0)
		else:
			print()

'''			
