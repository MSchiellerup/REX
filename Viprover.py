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

sleep(1)

def driveRobot(cm): #Sleep(4) for 100cm
	leftSpeed = 129
	speedConst = 1.315
	print("Driving %dcm",cm)
	print frindo.go_diff(leftSpeed, int(round(speedConst * leftSpeed)), 1, 1)	
	sleep(cm/25)

#driveRobot()
#sleep(4)

#print frindo.stop()
#sleep(1)
#print frindo.right()

sleep(1)

for x in range(1,5):
	driveRobot(100)
	#sleep(1)
	print frindo.right()

print frindo.stop()