from gpiozero import DistanceSensor
from gpiozero import Robot
from time import sleep
from signal import pause

cl=0.1
def inrange():
    global cl
    robot.stop()
    robot.backward(curve_right=0.5)
    sleep(2)
    #robot.right()
    #robot.reverse()
    sleep(1)
    robot.forward(curve_right=cl)

def outrange():
    global cl
    robot.stop()
    robot.forward(curve_right=cl)

sensor = DistanceSensor(echo=20, trigger=21, max_distance=1, threshold_distance=0.15)
sensor.when_in_range = inrange
sensor.when_out_of_range = outrange
#robot = Robot(left=(26, 19), right=(6, 13))
robot = Robot(right=(19, 26), left=(6, 13))

robot.forward(curve_right=cl)
pause()