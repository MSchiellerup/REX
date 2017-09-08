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


leftSpeed = 120
speedConst = 1.32
print frindo.go_diff(leftSpeed, int(round(speedConst * leftSpeed)), 1, 1)

sleep(4)

print frindo.stop()