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

# Set speed
print frindo.set_speed(70)
print frindo.set_turnspeed(20)
print frindo.set_step_time(1200)
print frindo.set_turn_time(1000)

# Set sensors
FrontSensor = 0
RightSensor = 0
LeftSensor = 0


# Boundaries
boundaries = [
	([29, 86, 6], 	 [64, 255, 255]), # green
	([17, 15, 100],  [50, 56, 200]),  # red
	([86, 31, 4], 	 [220, 88, 50]),  # blue
	([25, 146, 190], [62, 174, 250]), # yellow
	([103, 86, 65],  [145, 133, 128]) # gray
]

midx = 320
midy = 240

sleep(1)

def activateCam():
	camera = PiCamera()
	sleep(1) # Allow camera to start up
	camera.resolution = (640, 480)
	camera.framerate = 50

	camera.iso = 400

	camera.shutter_speed = camera.exposure_speed
	camera.exposure_mode = 'off'
	camera.brightness = 60

	rawCapture = PiRGBArray(camera, size=(640, 480))
	return camera, rawCapture

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
	if grader < 0:
		turn = round((1.8/360)*(-grader))
		print frindo.set_turnspeed(170)
		print frindo.left()
		sleep(0.01)
		print frindo.set_turnspeed(90)
		print frindo.left()
		sleep(turn)
	else:
		turn = round((1.8/360)*grader)
		print frindo.set_turnspeed(170)
		print frindo.right()
		sleep(0.01)
		print frindo.set_turnspeed(90)
		print frindo.right()
		sleep(turn)

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

def locateAndTurn():
	aveX = 0
	aveY = 0
	images = 0 
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		images = images + 1

		image = frame.array

		hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
		#blur = cv2.blur(image, (3,3))

		greenLower = np.array([29, 86, 6])
		greenUpper = np.array([64, 255, 255])

		thresh = cv2.inRange(hsv, greenLower, greenUpper)
		#thresh2 = thresh.copy()

		# find contours in the threshold image
		img, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

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
	  	aveY = aveY + cy
	  	aveX = aveX + cx

	  	key = cv2.waitKey(6) & 0xFF
	  	# clear the stream in preparation for the next frame
		rawCapture.truncate(0)
		if images == 5:
			break


	aveX = aveX / 5
	aveY = aveY / 5
	mRange = 30
	print("X = %d, Y = %d", aveX, aveY)
	if aveX < (midx-mRange) or aveX > (midx+mRange):
		if aveX < midx:
			#Turn left
			#turnRobot(-10)
			print "Drej venstre"
			return (-10)
		else:
			#turn right
			#turnRobot(10)
			print "Drej Hojre"
			return (10)
	return 0

def findBoxInFrame():
	camera, rawCapture = activateCam()

	camera.capture(rawCapture, format="bgr", use_video_port=True)
	frame = rawCapture.array

	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	greenLower = np.array([29, 86, 6])
    	greenUpper = np.array([64, 255, 255])

    	thresh = cv2.inRange(hsv, greenLower, greenUpper)

    	# find contours in the threshold image
	img, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

	max_area = 0
	best_cnt = 1
	for cnt in contours:
		area = cv2.contourArea(cnt)
		if area > max_area:
		    max_area = area
		    best_cnt = cnt

	# return false if it's not the box.
	# we could do this on the min size we expect the box to be?
	if max_area < 1:
		return (False, False)

	# find centroid of best_cnt and locate center
	M = cv2.moments(best_cnt)
	cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

	return (cx, cy), frame, contour

def convertBoxPositionToTurn(frame, boxPosition):
    height, width, channels = frame.shape
    x, y = boxPosition

    # let's assume that the left most pixels is 45 degrees to the left
    # then the right most pixel is also 45 degrees to the right
    # and we have in total 90 degrees to turn
    # To calcualte the turn degrees we simply get 90 * (x / width) - 45    
    return 90 * (x / width) - 45

def boxFound(contour):
    # replace 100 with some size
    return cv2.contourArea(cnt) > 100

def findBox():

    	foundBox = False
    	boxPosition = False

    	while not foundBox:
		x, y, frame, contour = findBoxInFrame()
		if (x+y > 0):
			boxPosition = (x,y)
			
        	print(boxPosition)
        
        	if not boxPosition:
    			# turn right 25 degrees and start the loop over
    			turnRobot(25)			
            		print("Turning 25 degrees. Loop")
            		continue

        	if boxFound(contour):
            		foundBox = True
            		continue

        	print("turn(%i)" % (convertBoxPositionToTurn(frame, boxPosition)))
        	print("drive(1)")

    	print('box found');
    	rawCapture.release()

findBox()
cv2.destroyAllWindows()

print frindo.stop()