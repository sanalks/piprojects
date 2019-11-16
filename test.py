from gpiozero import LED
from time import sleep
print("sanal")
led = LED(17)
while True:
    led.on()
    sleep(12)
    led.off()
    sleep(1)
