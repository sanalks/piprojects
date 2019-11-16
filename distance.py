from gpiozero import DistanceSensor
from time import sleep
from signal import pause

sensor = DistanceSensor(echo=20, trigger=21, max_distance=1, threshold_distance=0.2)

def inrange():
    print('Distance to nearest object is', sensor.distance, 'm')

sensor.when_in_range = inrange



pause()