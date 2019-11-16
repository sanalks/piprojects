from gpiozero import DistanceSensor
from gpiozero import Robot
from time import sleep
from signal import pause

robot = Robot(right=(19, 26), left=(6, 13))
robot.forward()
sleep(5)
robot.right(0.5)
sleep(5)
