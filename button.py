from gpiozero import Button, PWMLED
from time import sleep
from signal import pause
button = Button(2)
led = PWMLED(17)

led.pulse()

pause()


