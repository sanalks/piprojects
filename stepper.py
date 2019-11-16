from time import sleep
from gpiozero import OutputDevice



class StepperPin(OutputDevice):
    #comments
    #
    def __init__(
            self, pin=None, active_high=True, initial_value=False,
            pin_factory=None):
        self._blink_thread = None
        self._controller = None
        super(StepperPin, self).__init__(
            pin, active_high, initial_value, pin_factory=pin_factory
        )
    pass

pins = (StepperPin(6),StepperPin(13),StepperPin(19),StepperPin(26))
seq =[[1,0,0,0],
[1,1,0,0],
[0,1,0,0],
[0,1,1,0],
[0,0,1,0],
[0,0,1,1],
[0,0,0,1],
[1,0,0,1]]

seq =[[1,0,0,0],
[0,1,0,0],
[0,0,1,0],
[0,0,0,1]]

while True:
    for halfstep in range(len(seq)):
        sleep(0.004)
        for pin in range(4):
            if seq[halfstep][pin] == 1:
                pins[pin].on()
            else:
                pins[pin].off()