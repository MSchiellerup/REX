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
camera.resolution = (640, 480)
camera.framerate = 50
camera.hflip = True

rawCapture = PiRGBArray(camera, size=(640, 480))
 
# allow the camera to warmup
time.sleep(0.1)

# Boundaries
boundaries = [
	([29, 86, 6], 	 [64, 255, 255]), # green
	([17, 15, 100],  [50, 56, 200]),  # red
	([86, 31, 4], 	 [220, 88, 50]),  # blue
	([25, 146, 190], [62, 174, 250]), # yellow
	([103, 86, 65],  [145, 133, 128]) # gray
]

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	#Grab the image
	image = frame.array

	blur = cv2.blur(image, (3,3))

	greenLower = np.array([29, 86, 6])
	greenUpper = np.array([64, 255, 255])

	thresh = cv2.inRange(blur, greenLower, greenUpper)
	#thresh2 = thresh.copy()

	# find contours in the threshold image
	image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

	# finding contour with maximum area and store it as best_cnt
	max_area = 0
	best_cnt = 1
	for cnt in contours:
	 	area = cv2.contourArea(cnt)
	    	if area > max_area:
	      		max_area = area
	      		best_cnt = cnt


	  # finding centroids of best_cnt and draw a circle there
  	M = cv2.moments(best_cnt)
  	cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
	#if best_cnt>1:
	cv2.circle(blur,(cx,cy),10,(0,0,255),-1)
	# show the frame
	cv2.imshow("Frame", blur)
	print ("x = %d, y =  %d", cx, cy)
	#cv2.imshow('thresh',thresh2)
	key = cv2.waitKey(1) & 0xFF
	 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	 
		# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

print frindo.stop()