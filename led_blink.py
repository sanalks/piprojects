from gpiozero import PWMLED
from time import sleep

red = PWMLED(16)

while True:
    red.pulse()
