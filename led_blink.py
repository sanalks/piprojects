from gpiozero import LED
from time import sleep

red = LED(16)

while True:
    red.on()
    sleep(1)
    red.off()
    sleep(1)
