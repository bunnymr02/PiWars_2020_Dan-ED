# Code for Dan-ED - PiWars 2019 - v6.0 09/02/20
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
# triggered by pressing 'home' button, to break out of the loop.
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
#
# 05/02/20 - added code for ultrasonic sensors to measure and report distance to
# objects. Motor control code tidied by defining functions for left and right motor
# drive.
#
# 09/02/20 - added additional method for 'maze following'.  Based on the 'line
# following' code, but using signals from the left and front ultrasonic sensors.
# Dan-ED, essentially 'follows' the wall, keeping a preset distance from the left wall.


# Need floating point division of integers
#from __future_ import division
from time import sleep
import time

# Import GPIO (for MotoZero)
import RPi.GPIO as GPIO

# Needed for line follower
from gpiozero import Robot, LineSensor

left_sensor = LineSensor(26)
right_sensor = LineSensor(19)

left_detect = 1
right_detect = 1

pause_time = 0.0 # set to zero, but could be set to give Dan-ED more 'turn time'
max_speed = 60 # max speed in line follower mode
turn_speed = 100 # turn speed when Dan-ED strays from line
slow_speed = 20 # not used##

warning_distance = 15 # buffer distance to wall - 15cm

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)


# Define GPIO pins
Motor1A = 27 # left motor 1
Motor1B = 24 # left motor
Motor1Enable = 5 # left motor

Motor2A = 6 # left motor 2
Motor2B = 22 # left motor
Motor2Enable = 17 # left motor

# Define GPIO pins
Motor3A = 16 # right motor 3
Motor3B = 23 # right motor
Motor3Enable = 12 # right motor

Motor4A = 18 # right motor 4
Motor4B = 13 # right motor
Motor4Enable = 25 # right motor


# Set up defined GPIO pins
GPIO.setup(Motor1A,GPIO.OUT) # left motor 1
GPIO.setup(Motor1B,GPIO.OUT) # left motor
GPIO.setup(Motor1Enable,GPIO.OUT) # left motor on/off

GPIO.setup(Motor2A,GPIO.OUT) # left motor 2
GPIO.setup(Motor2B,GPIO.OUT) # left motor
GPIO.setup(Motor2Enable,GPIO.OUT) # left motor on/off

GPIO.setup(Motor3A,GPIO.OUT) # right motor 3
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

# Set up GPIO pins for ultrasonic sensors
GPIO_TRIGGER1 = 4       #GPIO 4 - Right ultrasonic sensor
GPIO_ECHO1 = 10         #GPIO 10 - Right ultrasonic sensor

GPIO_TRIGGER2 = 9       #GPIO 9 - Front ultrasonic sensor
GPIO_ECHO2 = 11         #GPIO 11 - Front ultrasonic sensor

GPIO_TRIGGER3 = 8       #GPIO 8 - Left ultrasonic sensor
GPIO_ECHO3 = 7          #GPIO 7 - Left ultrasonic sensor

# Set pins as output and input
GPIO.setup(GPIO_TRIGGER1,GPIO.OUT)  # Trigger
GPIO.setup(GPIO_ECHO1,GPIO.IN)      # Echo
GPIO.setup(GPIO_TRIGGER2,GPIO.OUT)  # Trigger
GPIO.setup(GPIO_ECHO2,GPIO.IN)      # Echo
GPIO.setup(GPIO_TRIGGER3,GPIO.OUT)  # Trigger
GPIO.setup(GPIO_ECHO3,GPIO.IN)      # Echo

# Set trigger to False (Low)
GPIO.output(GPIO_TRIGGER1, False)
GPIO.output(GPIO_TRIGGER2, False)
GPIO.output(GPIO_TRIGGER3, False)



def sonar(GPIO_TRIGGER,GPIO_ECHO):
    """
    Sends out and listens for ultrasonic 'PING'. Timing measurements are used to
    calculate distance travelled.

    """  
    start=0
    stop=0
    # Set pins as output and input
    GPIO.setup(GPIO_TRIGGER,GPIO.OUT)  # Trigger
    GPIO.setup(GPIO_ECHO,GPIO.IN)      # Echo

    # Set trigger to False (Low)
    GPIO.output(GPIO_TRIGGER, False)

    # Allow module to settle
    time.sleep(0.01)
       
    #while distance > 5:
    #Send 10us pulse to trigger
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    begin = time.time()
    while GPIO.input(GPIO_ECHO)==0 and time.time()<begin+0.05:
        start = time.time()

    while GPIO.input(GPIO_ECHO)==1 and time.time()<begin+0.1:
        stop = time.time()

    # Calculate pulse length
    elapsed = stop-start
    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound (cm/s)
    distance = elapsed * 34000

    # That was the distance there and back so halve the value
    distance = distance / 2

    #print("Distance : %.1f" % distance)
    #print("GPIO_TRIGGER:", GPIO_TRIGGER, "Distance: ", distance)
    # Reset GPIO settings
    return distance



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
    motor_multiplier = 1

    # measure distance - Left sensor
    #distanceL = sonar(GPIO_TRIGGER3,GPIO_ECHO3)


    def l_motor_forward(speed):

        GPIO.output(Motor1A,GPIO.HIGH) # GPIO high to power the + terminal
        GPIO.output(Motor1B,GPIO.LOW) # GPIO low to ground the - terminal
        #GPIO.output(Motor1Enable,GPIO.HIGH) # GPIO high to enable this motor           
        Motor1_PWM.start(speed) # PWM modulate enable pin
        GPIO.output(Motor2A,GPIO.HIGH) # GPIO high to power the + terminal
        GPIO.output(Motor2B,GPIO.LOW) # GPIO low to ground the - terminal
        #GPIO.output(Motor2Enable,GPIO.HIGH) # GPIO high to enable this motor
        Motor2_PWM.start(speed) # PWM modulate enable pin

    def r_motor_forward(speed):

        GPIO.output(Motor3A,GPIO.HIGH) # GPIO high to power the + terminal
        GPIO.output(Motor3B,GPIO.LOW) # GPIO low to ground the - terminal
        #GPIO.output(Motor3Enable,GPIO.HIGH) # GPIO high to enable this motor
        Motor3_PWM.start(speed) # PWM modulate enable pin
        GPIO.output(Motor4A,GPIO.HIGH) # GPIO high to power the + terminal
        GPIO.output(Motor4B,GPIO.LOW) # GPIO low to ground the - terminal
        #GPIO.output(Motor4Enable,GPIO.HIGH) # GPIO high to enable this motor
        Motor4_PWM.start(speed) # PWM modulate enable pin
    
    def l_motor_reverse(speed):
        GPIO.output(Motor1A,GPIO.LOW) # GPIO low to ground the - terminal
        GPIO.output(Motor1B,GPIO.HIGH) # GPIO high to power the + terminal
        #GPIO.output(Motor1Enable,GPIO.HIGH) # GPIO high to enable this motor
        Motor1_PWM.start(speed) # PWM modulate enable pin
        GPIO.output(Motor2A,GPIO.LOW) # GPIO low to ground the - terminal
        GPIO.output(Motor2B,GPIO.HIGH) # GPIO high to power the + terminal
        #GPIO.output(Motor2Enable,GPIO.HIGH) # GPIO high to enable this motor
        Motor2_PWM.start(speed) # PWM modulate enable pin
      
    def r_motor_reverse(speed):
        GPIO.output(Motor3A,GPIO.LOW) # GPIO low to ground the - terminal
        GPIO.output(Motor3B,GPIO.HIGH) # GPIO high to power the + terminal
        #GPIO.output(Motor3Enable,GPIO.HIGH) # GPIO high to enable this motor
        Motor3_PWM.start(speed) # PWM modulate enable pin
        GPIO.output(Motor4A,GPIO.LOW) # GPIO low to ground the - terminal
        GPIO.output(Motor4B,GPIO.HIGH) # GPIO high to power the + terminal
        #GPIO.output(Motor4Enable,GPIO.HIGH) # GPIO high to enable this motor  
        Motor4_PWM.start(speed) # PWM modulate enable pin

    def l_motor_stop():
        #GPIO.output(Motor2Enable,GPIO.LOW) # GPIO low to disable this motor
        #GPIO.output(Motor3Enable,GPIO.LOW) # GPIO low to disable this motor
        Motor1_PWM.stop()
        Motor2_PWM.stop()

    def r_motor_stop():        
        #GPIO.output(Motor1Enable,GPIO.LOW) # GPIO low to disable this motor
        #GPIO.output(Motor4Enable,GPIO.LOW) # GPIO low to disable this motor
        Motor3_PWM.stop()
        Motor4_PWM.stop()

    def left_too_close():
        #distance coming from right ultrasonic sensor
        distanceL = sonar(GPIO_TRIGGER3,GPIO_ECHO3)
        if distanceL < warning_distance:
            flag = 0
        else:
            flag = 1
        return flag

##
##    def right_too_close():
##
##        #distance coming from left ultrasonic sensor
##        distanceR = sonar(GPIO_TRIGGER1,GPIO_ECHO1)
##
##
##    def Front_too_close():
##
##        #distance coming from front ultrasonic sensor
##        distanceC = sonar(GPIO_TRIGGER2,GPIO_ECHO2)


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
            #distanceL = sonar(GPIO_TRIGGER3,GPIO_ECHO3)
            
            # If power is less than 0, turn motor backwards, otherwise turn it forwards
            if power_left < 0:
                l_motor_reverse(power_left*-1)
            elif power_left > 0:
                l_motor_forward(power_left)
            elif power_left == 0:
                l_motor_stop()

            if power_right < 0:
                r_motor_reverse(power_right*-1)
            elif power_right > 0:
                r_motor_forward(power_right)               
            elif power_right == 0:
                r_motor_stop()

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
            if 'square' in joystick.presses:
                print("Switch to Maze Follower Mode")
                start_maze_follow = 0
                set_maze_follower(start_maze_follow)
                


    def set_maze_follower(start_maze_follow):
        """
        This alternate flow is specific to the PiWars 2020 'Blind Maze' challenge.
        It picks up signals from the left, right and centre ultrasound sensors to guide
        Dan-ED through a maze of coloured walls which can be moved. The mode is selected
        from the main flow by pushing the 'square' button.

        Single push of l1 button starts Dan-ED rolling forward. Single push of r1 button
        stops Dan-ED.

        Output from left and front ultrasonic sensors are continuously read to provide
        a 'distance' readout.  Dan-ED essentially follows the 'left wall' at a preset
        'warning_distance' as determined by the left sensor.  If distanceL < warning_
        distance Dan-ED steers to the right by pausing the right motor.  If Dan_ED
        steers too far to the right, it will steer to the left by pausing the left motor
        The logic means Dan-ED 'follows' the wall defined by 'warning_distance'.  If the
        front sensor determines Dan-ED is about to hit a wall, it will sharply steer to
        the right by 'counter rotating' the tracks.
    
       
        """

        while joystick.connected:
            # distance coming from right ultrasonic sensor
            distanceR = sonar(GPIO_TRIGGER1,GPIO_ECHO1)
            # distance coming from front ultrasonic sensor
            distanceC = sonar(GPIO_TRIGGER2,GPIO_ECHO2)
            # distance coming from left ultrasonic sensor
            distanceL = sonar(GPIO_TRIGGER3,GPIO_ECHO3)
            # print("Left:", round(distanceL,1), "Centre:", round(distanceC,1), "Right:", round(distanceR,1))
           
            if start_maze_follow == 1:
                l_motor_forward(max_speed)
                r_motor_forward(max_speed)

                # if left sensor detects too close to wall, steer to right by pausing
                # the right motor  
                if distanceL < warning_distance:
                    l_motor_forward(85)
                    r_motor_stop()

                # if right sensor detects too close to wall, steer to left by pausing
                # the left motor  
                if distanceL > (warning_distance + 2):
                    r_motor_forward(85)
                    l_motor_stop()

                # if front sensor detects too close to wall, steer sharply to the right
                # by 'counter rotating'.  Left motor (forward), right motor (reverse).  
                if distanceC < (warning_distance + 2):
                    l_motor_forward(turn_speed)
                    r_motor_reverse(turn_speed)            

            elif start_maze_follow == 0:
                l_motor_stop()
                r_motor_stop()
               
            joystick.check_presses()
            # Print out any buttons that were pressed, if we had any
            if joystick.has_presses:
                print(joystick.presses)
            if 'l1' in joystick.presses:
                print("Forward - maze follower")
                start_maze_follow = 1
            if 'r1' in joystick.presses:
                print("Stop - maze follower")
                start_maze_follow = 0
            if 'r2' in joystick.presses:
                print("Switch to Manual Joystick Mode")
                power_left = 0 # stops it creeping forward
                power_right = 0 # stops it creeping forward
                set_manual_joystick(power_left, power_right)
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

           
            if start_line_follow == 1:
                l_motor_forward(max_speed)
                r_motor_forward(max_speed)

                # if vear to the left, set left motor (forward), right motor (reverse)
                # to steer right  
                if right_detect == 0:
                    l_motor_forward(turn_speed)
                    r_motor_reverse(turn_speed)

                # if vear to the right, set right motor (forward), left motor (reverse)
                # to steer left  
                if left_detect == 0:
                    r_motor_forward(turn_speed)
                    l_motor_reverse(turn_speed)              
                
            elif start_line_follow == 0:
                l_motor_stop()
                r_motor_stop()
               
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
            if 'square' in joystick.presses:
                print("Switch to Maze Follower Mode")
                start_maze_follow = 0
                set_maze_follower(start_maze_follow)


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
                    if 'square' in joystick.presses:
                        # print("square")
                        start_maze_follow = 1
                        set_maze_follower(start_maze_follow)   

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
