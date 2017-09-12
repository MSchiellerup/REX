import time
from time import sleep

import robot

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

def convertDistanceToCM(distance):
	distanceReturn = int(round((542.548958 * (0.967907915**distance))))
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
FrontSensor = frindo.read_front_ir_sensor()
print ("cm: %d", convertDistanceToCM(FrontSensor))


print frindo.stop()