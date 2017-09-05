import time
from time import sleep

import robot

# Create a robot object and initialize
frindo = robot.Robot()

print("Running...")

# Set speed
print frindo.set_speed(100)
print frindo.set_turnspeed(150)
print frindo.set_step_time(2000)
print frindo.set_turn_time(1500)

sleep(1)

print frindo.go()

sleep(5)

print frindo.stop()