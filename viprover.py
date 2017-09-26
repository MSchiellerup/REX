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


# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
time.sleep(1) # Wait for camera

camera.resolution = (640, 480)
camera.framerate = 30

camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'

gain = camera.awb_gains
camera.awb_mode='off'
#gain = (Fraction(2,1), Fraction(1,1))
#gain = (1.5, 1.5)
camera.awb_gains = gain

print "shutter_speed = ", camera.shutter_speed
print "awb_gains = ", gain

rawCapture = PiRGBArray(camera, size=camera.resolution)
 
# Open a window
WIN_RF = "PiCam";
cv2.namedWindow(WIN_RF);
cv2.moveWindow(WIN_RF, 100, 100);


# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image
	img = frame.array
 	
 	# threshold image
	ret, threshed_img = cv2.threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),
	                127, 255, cv2.THRESH_BINARY)
	# find contours and get the external one
	image, contours, hier = cv2.findContours(threshed_img, cv2.RETR_TREE,
	                cv2.CHAIN_APPROX_SIMPLE)
	 
	# with each contour, draw boundingRect in green
	# a minAreaRect in red and
	# a minEnclosingCircle in blue
	for c in contours:
	    # get the bounding rect
	    x, y, w, h = cv2.boundingRect(c)
	    # draw a green rectangle to visualize the bounding rect
	    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
	 
	    # get the min area rect
	    rect = cv2.minAreaRect(c)
	    box = cv2.boxPoints(rect)
	    # convert all coordinates floating point values to int
	    box = np.int0(box)
	    # draw a red 'nghien' rectangle
	    cv2.drawContours(img, [box], 0, (0, 0, 255))
	 
	    # finally, get the min enclosing circle
	    (x, y), radius = cv2.minEnclosingCircle(c)
	    # convert all values to int
	    center = (int(x), int(y))
	    radius = int(radius)
	    # and draw the circle in blue
	    img = cv2.circle(img, center, radius, (255, 0, 0), 2)
	 
	print(len(contours))
	cv2.drawContours(img, contours, -1, (255, 255, 0), 1)

	# show the frame
	cv2.imshow(WIN_RF, img)
	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break



print frindo.stop()