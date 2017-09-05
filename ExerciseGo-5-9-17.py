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

# Looping driving test 1
print("First test drive, go-stop-right-stop")
for x in range(1,5):
	print("Action %d", x)
	print frindo.go()
	sleep(3)
	print frindo.stop()
	sleep(2)
	print frindo.right()
	sleep(2)
	print frindo.stop()

# Looping driving test 2
#print("Second test drive, ")
#for x in range(1,5)
#	print("Action %d", x)
#	print frindo.go()
#	sleep(3)
#	print frindo.stop()
#	sleep(2)
#	print frindo.step_rotate_right()
#	sleep(2)
#	print frindo.stop()