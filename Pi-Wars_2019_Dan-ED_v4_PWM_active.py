# Code for Dan-ED - PiWars 2019 - v4.0 02/02/20
#
# By Dr. Kok-Fu Pang, based on code from Mike Horne and Tom Oinn/Emma Norling
#
# This code works with a Raspberry Pi to allow motors to be controlled using a wifi
# game pad controller. It also provides autonomous 'line following' capability using
# two line follower sensors to provide steering signals to Dan-ED.
#
# The main flow is towards the end of the code. There is a continuous loop formed by
# a 'while True' statement which picks up commands from the game pad controller. The
# loop is within a 'try' statement which allows an exception (RobotStopException) -
# triggered by pressing ‘home’ button, to break out of the loop.
#
# The main loop 'initiates' the controller and picks up joystick values (power_left,
# power_right) as well as game pad button presses and is purely used to determine
# which alternate flows to follow;
#
# 1) set_manual_joystick() - by moving the joystick or selecting R2 when in line
# follower mode.
# 2) set_line_follower() - by selecting L2 when in joystick mode, or at start up.
#
# When in line following mode, L1 is used to set Dan-ED rolling forward, it will then
# use signals from the line follower sensors to autonomously steer.  R1 is used to
# stop Dan-ED.
#
# 02/02/20 - line follower code has been reworked, following testing on the smooth
# surface of the arena.  Previous version worked well when following a smoothly
# curving white line, but could not turn quick enough when following a sharply
# bending line (as encountered in pi wars).  To make it respond quicker the code
# allows the tracks to counter rotate allowing Dan-ED to turn on the spot.

# Need floating point division of integers
#from __future_ import division
from time import sleep

# Import GPIO (for MotoZero)
import RPi.GPIO as GPIO

# Needed for line follower
from gpiozero import Robot, LineSensor

left_sensor = LineSensor(19)
right_sensor = LineSensor(26)

left_detect = 1
right_detect = 1

pause_time = 0.0 # set to zero, but could be set to give Dan-ED more 'turn time'
max_speed = 60 # max speed in line follower mode
half_speed = 100 # turn speed when Dan-ED strays from line
slow_speed = 20 # not used

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)


# Define GPIO pins
Motor1A = 27 # left motor 1 (switched with motor 2)
Motor1B = 24 # left motor
Motor1Enable = 5 # left motor

Motor2A = 6 # left motor 3
Motor2B = 22 # left motor
Motor2Enable = 17 # left motor

# Define GPIO pins
Motor3A = 16 # right motor 2 (switched with motor 1)
Motor3B = 23 # right motor
Motor3Enable = 12 # right motor

Motor4A = 18 # right motor 4
Motor4B = 13 # right motor
Motor4Enable = 25 # right motor


# Set up defined GPIO pins
GPIO.setup(Motor1A,GPIO.OUT) # left motor 1
GPIO.setup(Motor1B,GPIO.OUT) # left motor
GPIO.setup(Motor1Enable,GPIO.OUT) # left motor on/off

GPIO.setup(Motor2A,GPIO.OUT) # left motor 3
GPIO.setup(Motor2B,GPIO.OUT) # left motor
GPIO.setup(Motor2Enable,GPIO.OUT) # left motor on/off

GPIO.setup(Motor3A,GPIO.OUT) # right motor 2
GPIO.setup(Motor3B,GPIO.OUT) # right motor
GPIO.setup(Motor3Enable,GPIO.OUT) # right motor on/off

GPIO.setup(Motor4A,GPIO.OUT) # right motor 4
GPIO.setup(Motor4B,GPIO.OUT) # right motor
GPIO.setup(Motor4Enable,GPIO.OUT) # right motor on/off

# Define PWM speed control for the motors
Motor1_PWM = GPIO.PWM(Motor1Enable,50) # modulates motor speed using PWM - 50Hz
Motor2_PWM = GPIO.PWM(Motor2Enable,50) # modulates motor speed using PWM - 50Hz
Motor3_PWM = GPIO.PWM(Motor3Enable,50) # modulates motor speed using PWM - 50Hz
Motor4_PWM = GPIO.PWM(Motor4Enable,50) # modulates motor speed using PWM - 50Hz           


try:
    # Attempt to import the GPIO Zero library. If this fails, because we're running
    # somewhere that doesn't have the library, we create dummy functions for set_speeds
    # and stop_motors which just print out what they'd have done. This is a fairly
    # common way to deal with hardware that may or may not exist!

    # Turn the motor on

    # Use GPIO Zero implementation of CamJam EduKit robot (thanks Ben Nuttall/Dave Jones!)
    from gpiozero import CamJamKitRobot

    print('GPIO Zero found')

    # Get the robot instance and the independent motor controllers
    robot = CamJamKitRobot()
    motor_left = robot.left_motor
    motor_right = robot.right_motor

    # Motors are reversed. If you find your robot going backwards, set this to 1
    motor_multiplier = -1


    def set_manual_joystick(power_left, power_right):
        """

        This alternate flow picks up signals from the left joystick and is selected
        from the main flow.  Joystick signals are 'mixed' using the mixer function
        to generate a 'power_left' and 'power_right' signal in range 0 - 100. Depending
        on the power_left and power_right signals received, it is possible for Dan-ED
        to 'turn on the spot'.
        We have since implemented proportional speed control by applyng PWM to the
        motor enable pin.  The signal is derived from the 'mixed power_left and
        power_right signals.  Movement of Dan-ED is much more precise with PWM control.
       
        """

        while joystick.connected:
            # get line follower readings
            left_detect = int(left_sensor.value)
            right_detect = int(right_sensor.value)
       
            print("left: {}, right {}, line_left {}, line_right {}".format(power_left, power_right, left_detect, right_detect))
            power_left = (motor_multiplier * power_left)
            power_right = (motor_multiplier * power_right)

            # If power is less than 0, turn motor backwards, otherwise turn it forwards
            if power_left < 0:
                GPIO.output(Motor1A,GPIO.HIGH) # GPIO high to power the + terminal
                GPIO.output(Motor1B,GPIO.LOW) # GPIO low to ground the - terminal
                #GPIO.output(Motor1Enable,GPIO.HIGH) # GPIO high to enable this motor           
                Motor1_PWM.start(power_left*-1) # PWM modulate enable pin
                GPIO.output(Motor2A,GPIO.HIGH) # GPIO high to power the + terminal
                GPIO.output(Motor2B,GPIO.LOW) # GPIO low to ground the - terminal
                #GPIO.output(Motor2Enable,GPIO.HIGH) # GPIO high to enable this motor
                Motor2_PWM.start(power_left*-1) # PWM modulate enable pin
            elif power_left > 0:
                GPIO.output(Motor1A,GPIO.LOW) # GPIO low to ground the - terminal
                GPIO.output(Motor1B,GPIO.HIGH) # GPIO high to power the + terminal
                #GPIO.output(Motor1Enable,GPIO.HIGH) # GPIO high to enable this motor
                Motor1_PWM.start(power_left) # PWM modulate enable pin
                GPIO.output(Motor2A,GPIO.LOW) # GPIO low to ground the - terminal
                GPIO.output(Motor2B,GPIO.HIGH) # GPIO high to power the + terminal
                #GPIO.output(Motor2Enable,GPIO.HIGH) # GPIO high to enable this motor
                Motor2_PWM.start(power_left) # PWM modulate enable pin
            elif power_left == 0:
                #GPIO.output(Motor2Enable,GPIO.LOW) # GPIO low to disable this motor
                #GPIO.output(Motor3Enable,GPIO.LOW) # GPIO low to disable this motor
                Motor2_PWM.stop()
                Motor3_PWM.stop()
            if power_right < 0:
                GPIO.output(Motor3A,GPIO.HIGH) # GPIO high to power the + terminal
                GPIO.output(Motor3B,GPIO.LOW) # GPIO low to ground the - terminal
                #GPIO.output(Motor3Enable,GPIO.HIGH) # GPIO high to enable this motor
                Motor3_PWM.start(power_right*-1) # PWM modulate enable pin
                GPIO.output(Motor4A,GPIO.HIGH) # GPIO high to power the + terminal
                GPIO.output(Motor4B,GPIO.LOW) # GPIO low to ground the - terminal
                #GPIO.output(Motor4Enable,GPIO.HIGH) # GPIO high to enable this motor
                Motor4_PWM.start(power_right*-1) # PWM modulate enable pin
            elif power_right > 0:
                GPIO.output(Motor3A,GPIO.LOW) # GPIO low to ground the - terminal
                GPIO.output(Motor3B,GPIO.HIGH) # GPIO high to power the + terminal
                #GPIO.output(Motor3Enable,GPIO.HIGH) # GPIO high to enable this motor
                Motor3_PWM.start(power_right) # PWM modulate enable pin
                GPIO.output(Motor4A,GPIO.LOW) # GPIO low to ground the - terminal
                GPIO.output(Motor4B,GPIO.HIGH) # GPIO high to power the + terminal
                #GPIO.output(Motor4Enable,GPIO.HIGH) # GPIO high to enable this motor  
                Motor4_PWM.start(power_right) # PWM modulate enable pin
            elif power_right == 0:
                #GPIO.output(Motor1Enable,GPIO.LOW) # GPIO low to disable this motor
                #GPIO.output(Motor4Enable,GPIO.LOW) # GPIO low to disable this motor
                Motor1_PWM.stop()
                Motor4_PWM.stop()
            # Get joystick values from the left analogue stick
            x_axis, y_axis = joystick['lx', 'ly']
            # Get power from mixer function
            power_left, power_right = mixer(yaw=x_axis, throttle=y_axis)
            # Get a ButtonPresses object containing everything pressed since the last
            # time around this loop.
            joystick.check_presses()
            # Print out any buttons that were pressed, if we had any
            if joystick.has_presses:
                print(joystick.presses)
            if 'l2' in joystick.presses:
                print("Switch to Line Follower Mode")
                start_line_follow = 0
                set_line_follower(start_line_follow)


    def set_line_follower(start_line_follow):
        """
        This alternate flow picks up line follower signals.  It is selected from the
        main flow. Single push of l1 button starts Dan-ED rolling forward. Single
        push of r1 button stops Dan-ED. Output from left and right line follower
        sensors are continuously read.  On a black non reflective surface both sensors
        give a '1' (HIGH) output. If a reflective white line is detected it gives a
        '0' (LOW) output. Previously Line follower code momentarly paused left motors
        if left sensor detected '0', i.e., Dan-ED veering to the right and crossing
        reflective line.  If right sensor detected '0', i.e., Dan-ED veering to the
        left and crossing the line it would momentarily pause the right motor.

        Testing showed Dan-ED needed to steer more sharply.  Code changes tried to
        'slow down' Dan-ED on encountering a bend, but the best solution was to force
        Dan-ED to 'spin on the spot' by counter rotating the tracks.

        """

        while joystick.connected:
            # get line follower readings
            left_detect = int(left_sensor.value)
            right_detect = int(right_sensor.value)
            #print("line_left {}, line_right {}, start_line_follow: {}".format(left_detect, right_detect, start_line_follow))

            GPIO.output(Motor1A,GPIO.HIGH) # GPIO high to power the + terminal
            GPIO.output(Motor1B,GPIO.LOW) # GPIO low to ground the - terminal
            GPIO.output(Motor2A,GPIO.HIGH) # GPIO high to power the + terminal
            GPIO.output(Motor2B,GPIO.LOW) # GPIO low to ground the - terminal
            GPIO.output(Motor3A,GPIO.HIGH) # GPIO high to power the + terminal
            GPIO.output(Motor3B,GPIO.LOW) # GPIO low to ground the - terminal
            GPIO.output(Motor4A,GPIO.HIGH) # GPIO high to power the + terminal
            GPIO.output(Motor4B,GPIO.LOW) # GPIO low to ground the - terminal        
           
            if start_line_follow == 1:
                Motor1_PWM.start(max_speed) # PWM modulate enable pin               
                Motor2_PWM.start(max_speed) # PWM modulate enable pin
                Motor3_PWM.start(max_speed) # PWM modulate enable pin
                Motor4_PWM.start(max_speed) # PWM modulate enable pin

                # if vear to the left, set left motor (forward), right motor (reverse)
                # to steer right  
                if right_detect == 0:
                    GPIO.output(Motor1A,GPIO.LOW) # GPIO low to ground the - terminal
                    GPIO.output(Motor1B,GPIO.HIGH) # GPIO high to power the + terminal
                    GPIO.output(Motor2A,GPIO.LOW) # GPIO low to ground the - terminal
                    GPIO.output(Motor2B,GPIO.HIGH) # GPIO high to power the + terminal

                    GPIO.output(Motor3A,GPIO.HIGH) # GPIO high to power the + terminal
                    GPIO.output(Motor3B,GPIO.LOW) # GPIO low to ground the - terminal
                    GPIO.output(Motor4A,GPIO.HIGH) # GPIO high to power the + terminal
                    GPIO.output(Motor4B,GPIO.LOW) # GPIO low to ground the - terminal
                   
                    Motor1_PWM.start(half_speed) # PWM modulate enable pin
                    Motor2_PWM.start(half_speed) # PWM modulate enable pin
                    Motor3_PWM.start(half_speed) # PWM modulate enable pin
                    Motor4_PWM.start(half_speed) # PWM modulate enable pin
                    #sleep (pause_time) # more time to turn            
                    #Motor1_PWM.start(max_speed) # PWM modulate enable pin
                    #Motor2_PWM.start(max_speed) # PWM modulate enable pin
                    #Motor3_PWM.start(max_speed) # PWM modulate enable pin
                    #Motor4_PWM.start(max_speed) # PWM modulate enable pin

                # if vear to the right, set right motor (forward), left motor (reverse)
                # to steer left  
                if left_detect == 0:
                    GPIO.output(Motor1A,GPIO.HIGH) # GPIO high to power the + terminal
                    GPIO.output(Motor1B,GPIO.LOW) # GPIO low to ground the - terminal
                    GPIO.output(Motor2A,GPIO.HIGH) # GPIO high to power the + terminal
                    GPIO.output(Motor2B,GPIO.LOW) # GPIO low to ground the - terminal

                    GPIO.output(Motor3A,GPIO.LOW) # GPIO high to power the + terminal
                    GPIO.output(Motor3B,GPIO.HIGH) # GPIO low to ground the - terminal
                    GPIO.output(Motor4A,GPIO.LOW) # GPIO high to power the + terminal
                    GPIO.output(Motor4B,GPIO.HIGH) # GPIO low to ground the - terminal

                    Motor1_PWM.start(half_speed) # PWM modulate enable pin
                    Motor2_PWM.start(half_speed) # PWM modulate enable pin
                    Motor3_PWM.start(half_speed) # PWM modulate enable pin
                    Motor4_PWM.start(half_speed) # PWM modulate enable pin
                    #sleep (pause_time) # more time to turn
                    #Motor1_PWM.start(max_speed) # PWM modulate enable pin
                    #Motor2_PWM.start(max_speed) # PWM modulate enable pin
                    #Motor3_PWM.start(max_speed) # PWM modulate enable pin
                    #Motor4_PWM.start(max_speed) # PWM modulate enable pin
                
            elif start_line_follow == 0:
               Motor1_PWM.stop()               
               Motor2_PWM.stop()
               Motor3_PWM.stop()
               Motor4_PWM.stop()
               
            joystick.check_presses()
            # Print out any buttons that were pressed, if we had any
            if joystick.has_presses:
                print(joystick.presses)
            if 'l1' in joystick.presses:
                print("Forward - line follower")
                start_line_follow = 1
            if 'r1' in joystick.presses:
                print("Stop - line follower")
                start_line_follow = 0

            if 'r2' in joystick.presses:
                print("Switch to Manual Joystick Mode")
                power_left = 0 # stops it creeping forward
                power_right = 0 # stops it creeping forward
                set_manual_joystick(power_left, power_right)


    def stop_motors():
        """
        As we have an motor hat, stop the motors using their motors call
        """
        # Turn both motors off
        motor_left.stop()
        motor_right.stop()

except ImportError:

    print('GPIO Zero not found, using dummy functions.')


    def set_speeds(power_left, power_right):
        """
        No motor hat - print what we would have sent to it if we'd had one.
        """
        print('DEBUG Left: {}, Right: {}'.format(power_left, power_right))
        sleep(0.3)


    def stop_motors():
        """
        No motor hat, so just print a message.
        """
        print('DEBUG Motors stopping')

# All we need, as we don't care which controller we bind to, is the ControllerResource
from approxeng.input.selectbinder import ControllerResource


# Enable logging of debug messages, by default these aren't shown
# import logzero
# logzero.setup_logger(name='approxeng.input', level=logzero.logging.DEBUG)

class RobotStopException(Exception):
    """
    The simplest possible subclass of Exception, we'll raise this if we want to stop
    the robot for any reason. Creating a custom exception like this makes the code
    more readable later.
    """
    pass


def mixer(yaw, throttle, max_power=100):
    """
    Mix a pair of joystick axes, returning a pair of wheel speeds. This is where the
    mapping from joystick positions to wheel powers is defined, so any changes to
    how the robot drives should be made here, everything else is really just plumbing.
        
    :param yaw: 
        Yaw axis value, ranges from -1.0 to 1.0
    :param throttle: 
        Throttle axis value, ranges from -1.0 to 1.0
    :param max_power: 
        Maximum speed that should be returned from the mixer, defaults to 100
    :return: 
        A pair of power_left, power_right integer values to send to the motor driver
    """
    left = throttle + yaw
    right = throttle - yaw
    scale = float(max_power) / max(1, abs(left), abs(right))
    return int(left * scale), int(right * scale)


# Outer try / except catches the RobotStopException we just defined, which we'll
# raise when we want to bail out of the loop cleanly, shutting the motors down. We
# can raise this in response to a button press
try:
    while True:
        # Inner try / except is used to wait for a controller to become available, at
        # which point we bind to it and enter a loop where we read axis values and
        # send commands to the motors.
        try:
            # Bind to any available joystick, this will use whatever's connected as
            # long as the library supports it.
            with ControllerResource(dead_zone=0.1, hot_zone=0.2) as joystick:
                print('Controller found, press HOME button to exit, use left stick to drive.')
                print(joystick.controls)
                # Loop until the joystick disconnects, or we deliberately stop by
                # raising a RobotStopException
                while joystick.connected:
                    # Get joystick values from the left analogue stick
                    x_axis, y_axis = joystick['lx', 'ly']
                    # Get power from mixer function
                    power_left, power_right = mixer(yaw=x_axis, throttle=y_axis)
                    # Set joystick mode
                    set_manual_joystick(power_left, power_right)
                    # Get a ButtonPresses object containing everything that was pressed
                    # since the last time around this loop.
                    joystick.check_presses()
                    # Print out any buttons that were pressed, if we had any
                    if joystick.has_presses:
                        print(joystick.presses)
                    if 'l1' in joystick.presses:
                        # print("help")
                        start_line_follow = 1
                        set_line_follower(start_line_follow)
                    if 'r2' in joystick.presses:
                        # print("help")
                        start_line_follow = 0
                        set_line_follower(start_line_follow)                        
                    # If home was pressed, raise a RobotStopException to bail out of
                    # the loop Home is generally the PS button for playstation
                    # controllers, XBox for XBox etc
                    if 'home' in joystick.presses:
                        raise RobotStopException()
                    
        except IOError:
            # We get an IOError when using the ControllerResource if we don't have a
            # controller yet, so in this case we just wait a second and try again after
            # printing a message.
            print('No controller found yet')
            sleep(1)
except RobotStopException:
    # This exception will be raised when the home button is pressed, at which point we
    # should stop the motors.
    stop_motors()

GPIO.cleanup() 
