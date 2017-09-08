import time
from time import sleep

import robot

# Create a robot object and initialize
frindo = robot.Robot()

print("Running...")

# Set speed
print frindo.set_speed(70)
print frindo.set_turnspeed(150)
print frindo.set_step_time(2000)
print frindo.set_turn_time(1000)

sleep(1)

def driveRobot():
	leftSpeed = 129
	speedConst = 1.315
	print frindo.go_diff(leftSpeed, int(round(speedConst * leftSpeed)), 1, 1)	

print frindo.step_rotate_right()

sleep(1)

print frindo.stop()