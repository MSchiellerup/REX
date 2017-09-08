import time
from time import sleep

import robot

# Create a robot object and initialize
frindo = robot.Robot()

print("Running...")

# Set speed
print frindo.set_speed(70)
print frindo.set_turnspeed(130)
print frindo.set_step_time(1200)
print frindo.set_turn_time(1000)

sleep(1)

def driveRobot(): #Sleep(4) for 100cm
	leftSpeed = 129
	speedConst = 1.315
	print frindo.go_diff(leftSpeed, int(round(speedConst * leftSpeed)), 1, 1)	

driveRobot()
sleep(4)

print frindo.stop()
#sleep(1)
print frindo.right()

sleep(0.9)

print frindo.stop()