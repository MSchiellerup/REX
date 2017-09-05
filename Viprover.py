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

# request to read Front IR sensor (analog sensor 0)
print "Front sensor = ", frindo.read_front_ir_sensor()

# request to read Right IR sensor (analog sensor 1)
print "Right sensor = ", frindo.read_right_ir_sensor()

# request to read Left IR sensor (analog sensor 2)
print "Left sensor = ", frindo.read_left_ir_sensor()



# send a go_diff command to drive forward
#leftSpeed = 128
#speedConst = 1.3
#print frindo.go_diff(leftSpeed, int(round(speedConst * leftSpeed)), 1, 1)

# Wait a bit while robot moves forward
#sleep(5)

#print frindo.stop()

#sleep(2)
for x in range(1,4):
	print frindo.right()
	sleep(0.53)
	print frindo.stop()
	sleep(1)

# send a stop command
print frindo.stop()