from gpiozero import DigitalOutputDevice
from gpiozero import LED
from time import sleep
from gpiozero import OutputDeviceBadValue, GPIOPinMissing
from gpiozero import GPIODevice, Device, CompositeDevice
from gpiozero import SourceMixin
from gpiozero import DistanceSensor

class StepperMotor(DigitalOutputDevice):
    def __init__(
            self, stepper_pin=None, direction_pin=None, pin_factory=None):
        #self._blink_thread = None
        #self._controller = None
        if not all(p is not None for p in [stepper_pin, direction_pin]):
            raise GPIOPinMissing(
                'forward and backward pins must be provided'
            )
        super(StepperMotor, self).__init__(
            stepper_pin, direction_pin, pin_factory=pin_factory
        )
        try:
            # XXX need a way of setting these together
            self.direction = LED(direction_pin)
            #self.value = initial_value
        except:
            self.close()
            raise
    
    def forward(self, speed=1):
        self.direction.on()
        on_time_max=0.002
        off_time_max=0.001
        if(speed <= 0.0): speed =0.0001
        if(speed > 1.0): speed = 1.0
        wave_length = on_time_max+off_time_max
        total_time = wave_length/speed
        self.blink(on_time=total_time-off_time_max, off_time=off_time_max)

    def backward(self, speed=1):
        self.direction.off()
        on_time_max=0.002
        off_time_max=0.001
        if(speed <= 0.0): speed =0.0001
        if(speed > 1.0): speed = 1.0
        wave_length = on_time_max+off_time_max
        total_time = wave_length/speed
        self.blink(on_time=total_time-off_time_max, off_time=off_time_max)

class StepperRobot(SourceMixin, CompositeDevice):
    """
    Extends :class:`CompositeDevice` to represent a generic dual-motor robot.

    This class is constructed with two tuples representing the forward and
    backward pins of the left and right controllers respectively. For example,
    if the left motor's controller is connected to GPIOs 4 and 14, while the
    right motor's controller is connected to GPIOs 17 and 18 then the following
    example will drive the robot forward::

        from gpiozero import Robot

        robot = Robot(left=(4, 14), right=(17, 18))
        robot.forward()

    :param tuple left:
        A tuple of two GPIO pins representing the forward and backward inputs
        of the left motor's controller.

    :param tuple right:
        A tuple of two GPIO pins representing the forward and backward inputs
        of the right motor's controller.

    :param Factory pin_factory:
        See :doc:`api_pins` for more information (this is an advanced feature
        which most users can ignore).
    """

    def __init__(self, left=None, right=None, pin_factory=None):
        super(StepperRobot, self).__init__(
            left_motor=StepperMotor(*left, pin_factory=pin_factory),
            right_motor=StepperMotor(*right, pin_factory=pin_factory),
            _order=('left_motor', 'right_motor'),
            pin_factory=pin_factory
        )

    @property
    def value(self):
        """
        Represents the motion of the robot as a tuple of (left_motor_speed,
        right_motor_speed) with ``(-1, -1)`` representing full speed backwards,
        ``(1, 1)`` representing full speed forwards, and ``(0, 0)``
        representing stopped.
        """
        return super(StepperRobot, self).value

    @value.setter
    def value(self, value):
        self.left_motor.value, self.right_motor.value = value

    def forward(self, speed=1, **kwargs):
        """
        Drive the robot forward by running both motors forward.

        :param float speed:
            Speed at which to drive the motors, as a value between 0 (stopped)
            and 1 (full speed). The default is 1.

        :param float curve_left:
            The amount to curve left while moving forwards, by driving the
            left motor at a slower speed. Maximum ``curve_left`` is 1, the
            default is 0 (no curve). This parameter can only be specified as a
            keyword parameter, and is mutually exclusive with ``curve_right``.

        :param float curve_right:
            The amount to curve right while moving forwards, by driving the
            right motor at a slower speed. Maximum ``curve_right`` is 1, the
            default is 0 (no curve). This parameter can only be specified as a
            keyword parameter, and is mutually exclusive with ``curve_left``.
        """
        curve_left = kwargs.pop('curve_left', 0)
        curve_right = kwargs.pop('curve_right', 0)
        if kwargs:
            raise TypeError('unexpected argument %s' % kwargs.popitem()[0])
        if not 0 <= curve_left <= 1:
            raise ValueError('curve_left must be between 0 and 1')
        if not 0 <= curve_right <= 1:
            raise ValueError('curve_right must be between 0 and 1')
        if curve_left != 0 and curve_right != 0:
            raise ValueError('curve_left and curve_right can\'t be used at the same time')
        self.left_motor.forward(speed * (1 - curve_left))
        self.right_motor.backward(speed * (1 - curve_right))

    def backward(self, speed=1, **kwargs):
        """
        Drive the robot backward by running both motors backward.

        :param float speed:
            Speed at which to drive the motors, as a value between 0 (stopped)
            and 1 (full speed). The default is 1.

        :param float curve_left:
            The amount to curve left while moving backwards, by driving the
            left motor at a slower speed. Maximum ``curve_left`` is 1, the
            default is 0 (no curve). This parameter can only be specified as a
            keyword parameter, and is mutually exclusive with ``curve_right``.

        :param float curve_right:
            The amount to curve right while moving backwards, by driving the
            right motor at a slower speed. Maximum ``curve_right`` is 1, the
            default is 0 (no curve). This parameter can only be specified as a
            keyword parameter, and is mutually exclusive with ``curve_left``.
        """
        curve_left = kwargs.pop('curve_left', 0)
        curve_right = kwargs.pop('curve_right', 0)
        if kwargs:
            raise TypeError('unexpected argument %s' % kwargs.popitem()[0])
        if not 0 <= curve_left <= 1:
            raise ValueError('curve_left must be between 0 and 1')
        if not 0 <= curve_right <= 1:
            raise ValueError('curve_right must be between 0 and 1')
        if curve_left != 0 and curve_right != 0:
            raise ValueError('curve_left and curve_right can\'t be used at the same time')
        self.left_motor.backward(speed * (1 - curve_left))
        self.right_motor.forward(speed * (1 - curve_right))

    def left(self, speed=1):
        """
        Make the robot turn left by running the right motor forward and left
        motor backward.

        :param float speed:
            Speed at which to drive the motors, as a value between 0 (stopped)
            and 1 (full speed). The default is 1.
        """
        self.right_motor.forward(speed)
        self.left_motor.forward(speed)

    def right(self, speed=1):
        """
        Make the robot turn right by running the left motor forward and right
        motor backward.

        :param float speed:
            Speed at which to drive the motors, as a value between 0 (stopped)
            and 1 (full speed). The default is 1.
        """
        self.left_motor.backward(speed)
        self.right_motor.backward(speed)

    def reverse(self):
        """
        Reverse the robot's current motor directions. If the robot is currently
        running full speed forward, it will run full speed backward. If the
        robot is turning left at half-speed, it will turn right at half-speed.
        If the robot is currently stopped it will remain stopped.
        """
        self.left_motor.reverse()
        self.right_motor.reverse()

    def stop(self):
        """
        Stop the robot.
        """
        self.left_motor.stop()
        self.right_motor.stop()

stepRobot = StepperRobot(right=(12,16), left=(21,20))
# stepRobot.forward()
# stepper = StepperMotor(stepper_pin=16,direction_pin=12)
sensor = DistanceSensor(echo=4, trigger=17, max_distance=1, threshold_distance=0.2)
speed=0.9
forward =True
changed=True
def inrange():
    if forward== True :
        changed =True
        forward = False
    else:
        changed = False
    print('Distance to nearest object is', sensor.distance, 'm')
def outrange():
    if forward == True:
        changed=False
    else:
        changed=True
        forward=True
    print('Distance to nearest object is', sensor.distance, 'm')

sensor.when_in_range = inrange
sensor.when_out_of_range = outrange

while True:
    """ if changed == True:
        if forward == True :
            stepRobot.forward(speed)
        else :
            stepRobot.backward(speed)
        sleep(1)
        changed = False """

    for i in range(1,20):
        stepRobot.forward(speed)
        sleep(1)
    
    stepRobot.backward(speed)
    sleep(60)

    