import time
from time import sleep

import robot

from picamera.array import PiRGBArray
from picamera import PiCamera
from fractions import *
import cv2
import numpy as np

# Create a robot object and initialize
frindo = robot.Robot()

print("Running...")

# Set speed
print frindo.set_speed(70)
print frindo.set_turnspeed(121)
print frindo.set_step_time(1200)
print frindo.set_turn_time(1000)


FrontSensor = 0
RightSensor = 0
LeftSensor = 0

sleep(1)

def convertFrontDistanceToCM(distance):
	distanceReturn = int(round((79.943427 * (0.9953660709 ** distance))))
	return distanceReturn

def convertRightDistanceToCM(distance):
	distanceReturn = int(round((85.60176169 * (0.995149692 ** distance))))
	return distanceReturn

def convertLeftDistanceToCM(distance):
	distanceReturn = int(round((90.5118426 * (0.9951333544 ** distance))))
	return distanceReturn

def driveRobot(cm): #Sleep(4) for 100cm
	leftSpeed = 129
	speedConst = 1.305
	print("Driving %dcm",cm)
	print frindo.go_diff(leftSpeed, int(round(speedConst * leftSpeed)), 1, 1)	
	sleep(cm/25)

def turnRobot(grader):
	if grader > 0:
		print frindo.right()
		sleep((2.4/360)*grader)
	else:
		print frindo.left()
		sleep((2.4/360)*(-grader))

def driveSquared():
	for x in range(1,5):
		driveRobot(100)
		turnRobot(90)

def driveNumber8():
	speedConst = 1.305
	leftSpeed = 120
	print frindo.go_diff(leftSpeed/2, int(round(speedConst * leftSpeed)), 1, 1)	#venstre
	sleep(2)
	print frindo.go_diff(leftSpeed, int(round((speedConst * leftSpeed)/2)), 1, 1)	#hojre
	sleep(2)
	print frindo.go_diff(leftSpeed, int(round((speedConst * leftSpeed)/2)), 1, 1)	#hojre
	sleep(2)
	#driveRobot(50)
	print frindo.go_diff(leftSpeed/2, int(round(speedConst * leftSpeed)), 1, 1)	#venstre
	sleep(2)

def driveToStop():
	while frindo.read_front_ir_sensor() < 250 and frindo.read_right_ir_sensor() < 250 and frindo.read_left_ir_sensor() < 245:
		driveRobot(5)

def driveAround():
	for x in range(0,10):
		driveToStop()
		RightSensor = frindo.read_right_ir_sensor()
		LeftSensor = frindo.read_left_ir_sensor()
		if RightSensor < LeftSensor:
			turnRobot(90)
		else:
			turnRobot(-90)

#def detectGreenBox():
#def activateCam():
	# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
time.sleep(1) # Wait for camera

# Setting resolution and fr for camera
camera.resolution = (640, 480)
camera.framerate = 30

# get the current center of the image
#int midx = 320
#int midy = 240

camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'

gain = camera.awb_gains
camera.awb_mode='off'

camera.awb_gains = gain

print "shutter_speed = ", camera.shutter_speed
print "awb_gains = ", gain

rawCapture = PiRGBArray(camera, size=camera.resolution)
 
# Open a window
WIN_RF = "PiCam";
cv2.namedWindow(WIN_RF);
cv2.moveWindow(WIN_RF, 100, 100);

# define the list of boundaries
boundaries = [
	([29, 86, 6], 	 [64, 255, 255]), # green
	([17, 15, 100],  [50, 56, 200]),  # red
	([86, 31, 4], 	 [220, 88, 50]),  # blue
	([25, 146, 190], [62, 174, 250]), # yellow
	([103, 86, 65],  [145, 133, 128]) # gray
]

greenLower = np.array([29, 86, 6])
greenUpper = np.array([64, 255, 255])

# allow the camera to warmup
time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image
	image = frame.array
	# Convert to HSV
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	# Threshhold hsv to only get green colour
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	# Bitwise-AND mask and original image
  res = cv2.bitwise_and(image,image, mask = mask)

 # capture frames from the camera


	
	#mask = cv2.inRange(image, greenLower, greenUpper)
	#output = cv2.bitwise_and(image, image, mask = mask)
 # show the frame
	cv2.imshow(WIN_RF, np.hstack([image, res]))
	key = cv2.waitKey(4) & 0xFF

	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break







print frindo.stop()