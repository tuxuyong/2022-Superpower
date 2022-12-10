# LEGO type:standard slot:8 autostart

# New idea to combine Trip 2 and Trip 3
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
import math

hub = PrimeHub()

hub.light_matrix.show_image('HAPPY')

LeftMotorSym = 'F'
RightMotorSym = 'A'
LeftColorSym = 'E'
RightColorSym = 'B'

Hub = PrimeHub()
MotionSensor = MotionSensor()
motor_pair = MotorPair(LeftMotorSym, RightMotorSym)
LeftMotor = Motor(LeftMotorSym)
RightMotor = Motor(RightMotorSym)
LeftArm = Motor('D')
LeftArm.set_stop_action("hold")
RightArm = Motor('C')
LeftColor = ColorSensor(LeftColorSym)
RightColor = ColorSensor(RightColorSym)

# WHEEL_DIAMETER = 8.8 large wheel
WHEEL_DIAMETER = 6.24 # medium wheel
ANGLE2DIS = WHEEL_DIAMETER * math.pi / 360
WHITE = 98
BLACK = 18


def initialize():
    '''
    Initialize robot by
    - reset yaw angle
    - set stop action to hold
    '''
    MotionSensor.reset_yaw_angle()
    motor_pair.set_stop_action('hold')

def wait():
    '''
    wait for left button is pressed
    '''
    hub.light_matrix.show_image('HAPPY')
    Hub.left_button.wait_until_pressed()

def done():
    '''
    Trip is done, show happy face
    '''
    hub.light_matrix.show_image('HAPPY')

def trip_inprogress():
    hub.light_matrix.show_image('DIAMOND')

def on_target(target, allowance):
    '''
    Define if yaw angle has earched target

    Parameters
    target: target gyro angle
    allowance: error margin
    '''
    cur = MotionSensor.get_yaw_angle()
    res = target - cur
    if res > 180:
        res = res - 360
    elif res < -180:
        res = 360 + res
    return (True, res) if res >= -allowance and res <= allowance else (False, res)


def StraightPID_left(degree_abs, dist, speed, slow_stop=1, Kp=0.8, Ki=0.05, Kd=0.4):
    '''
    commissioning edition StraightPID program
    move straight following specific absolute gyro degree. It use speed to adjust direction

    Parameters
    degree_abs: absolute gyro degree to follow
    dist: distance to travel in cm
    speed:speed to travel, 0-100
    slow_stop: flag to define if slow stop will be applied at end of travel
            default is true
    Kp: Kp PID parameter, default 0.4
    Ki: Ki PID parameter, default 0.005
    Kd: Kd PID parameter, default 0.8
    '''
    global motor_pair

    count = 0
    count_temp = 0
    total_dist = 0
    degree_temp = 0
    degree_pre = 0
    a_Error = 0
    Pre_Error = 0
    RightMotor.set_degrees_counted(0)
    MaxSpeed = math.floor(speed * 5)

    degree_abs = degree_abs % 360
    Yaw = MotionSensor.get_yaw_angle()
    # Convert input from 360 scale to +/-180 scale
    if degree_abs > 179:
        degree_abs = degree_abs - 360
    else:
        degree_abs = degree_abs

    # Calculate Error between two +/-180 scale values
    Error = degree_abs - Yaw
    if Error > 180:
        Error = Error - 360
    elif Error < -180:
        Error = Error + 360

    if dist < 0:
        motor_pair = MotorPair(RightMotorSym, LeftMotorSym)
    else:
        motor_pair = MotorPair(LeftMotorSym, RightMotorSym)
    motor_pair.set_stop_action('hold')

    while total_dist < abs(dist):
        a_Error = a_Error * 0.75 + Error
        d_Error = Error - Pre_Error
        Pre_Error = Error
        C_P = Kp * Error
        C_I = Ki * a_Error
        C_D = Kd * d_Error
        C_Turn = C_P + C_I + C_D
        #C_Turn = C_Turn * 1
        if C_Turn > 0:
            C_Speed_Offset = math.ceil(C_Turn)
            C_Speed_Offset_slow = math.ceil(C_Turn*0.5)
        else:
            C_Speed_Offset = math.floor(C_Turn)
            C_Speed_Offset_slow = math.floor(C_Turn*0.5)

        # Slow Start
        if total_dist <= 0.5:
            motor_pair.start_tank(20 + C_Speed_Offset_slow, 20)
        else:
            if slow_stop == 1:
                if abs(dist) - total_dist > 2:
                    motor_pair.start_tank(speed + C_Speed_Offset, speed)
                else:
                    motor_pair.start_tank(15 + C_Speed_Offset_slow, 15)
            else:
                motor_pair.start_tank(speed + C_Speed_Offset, speed)

        # in case robot got stuck
        degree_temp = abs(RightMotor.get_degrees_counted())
        total_dist = degree_temp * ANGLE2DIS
        if count % 10 == 0 and total_dist > 2:
            if degree_temp - degree_pre < 1 :
                count_temp = count_temp + 1
                degree_pre = degree_temp
            else:
                degree_pre = degree_temp

        if count_temp > 3:
            break

        count = count + 1
        Yaw = MotionSensor.get_yaw_angle()
        Error = degree_abs - Yaw
        if Error > 180:
            Error = Error - 360
        elif Error < -180:
            Error = Error + 360

    motor_pair.stop()
    motor_pair = MotorPair(LeftMotorSym, RightMotorSym)

def StraightPID_right(degree_abs, dist, speed, slow_stop=1, Kp=0.8, Ki=0.05, Kd=0.4):
    '''
    commissioning edition StraightPID program
    move straight following specific absolute gyro degree. It use speed to adjust direction

    Parameters
    degree_abs: absolute gyro degree to follow
    dist: distance to travel in cm
    speed:speed to travel, 0-100
    slow_stop: flag to define if slow stop will be applied at end of travel
            default is true
    Kp: Kp PID parameter, default 0.4
    Ki: Ki PID parameter, default 0.005
    Kd: Kd PID parameter, default 0.8
    '''
    global motor_pair

    count = 0
    count_temp = 0
    total_dist = 0
    degree_temp = 0
    degree_pre = 0
    a_Error = 0
    Pre_Error = 0
    LeftMotor.set_degrees_counted(0)
    MaxSpeed = math.floor(speed * 5)

    degree_abs = degree_abs % 360
    Yaw = MotionSensor.get_yaw_angle()
    # Convert input from 360 scale to +/-180 scale
    if degree_abs > 179:
        degree_abs = degree_abs - 360
    else:
        degree_abs = degree_abs

    # Calculate Error between two +/-180 scale values
    Error = degree_abs - Yaw
    if Error > 180:
        Error = Error - 360
    elif Error < -180:
        Error = Error + 360

    if dist < 0:
        motor_pair = MotorPair(RightMotorSym, LeftMotorSym)
    else:
        motor_pair = MotorPair(LeftMotorSym, RightMotorSym)
    motor_pair.set_stop_action('hold')

    while total_dist < abs(dist):
        a_Error = a_Error * 0.75 + Error
        d_Error = Error - Pre_Error
        Pre_Error = Error
        C_P = Kp * Error
        C_I = Ki * a_Error
        C_D = Kd * d_Error
        C_Turn = C_P + C_I + C_D
        #C_Turn = C_Turn * 1
        if C_Turn > 0:
            C_Speed_Offset = math.ceil(C_Turn)
            C_Speed_Offset_slow = math.ceil(C_Turn*0.5)
        else:
            C_Speed_Offset = math.floor(C_Turn)
            C_Speed_Offset_slow = math.floor(C_Turn*0.5)

        # Slow Start
        if total_dist <= 0.5:
            motor_pair.start_tank(20, 20 - C_Speed_Offset_slow)
        else:
            if slow_stop == 1:
                if abs(dist) - total_dist > 2:
                    #motor_pair.start_tank(min(speed + C_Speed_Offset, MaxSpeed), speed)
                    motor_pair.start_tank(speed , speed - C_Speed_Offset)
                else:
                    motor_pair.start_tank(15, 15 - C_Speed_Offset_slow)
            else:
                motor_pair.start_tank(speed, speed - C_Speed_Offset)

        # in case robot got stuck
        degree_temp = abs(LeftMotor.get_degrees_counted())
        total_dist = degree_temp * ANGLE2DIS
        if count % 10 == 0 and total_dist > 2:
            if degree_temp - degree_pre < 1 :
                count_temp = count_temp + 1
                degree_pre = degree_temp
            else:
                degree_pre = degree_temp

        if count_temp > 3:
            break

        count = count + 1
        Yaw = MotionSensor.get_yaw_angle()
        Error = degree_abs - Yaw
        if Error > 180:
            Error = Error - 360
        elif Error < -180:
            Error = Error + 360

    motor_pair.stop()
    motor_pair = MotorPair(LeftMotorSym, RightMotorSym)

def StraightPID_double(degree_abs, dist, speed, slow_stop=1, Kp=1, Ki=0.05, Kd=0.5):
    '''
    commissioning edition StraightPID program
    move straight following specific absolute gyro degree. It use speed to adjust direction

    Parameters
    degree_abs: absolute gyro degree to follow
    dist: distance to travel in cm
    speed:speed to travel, 0-100
    slow_stop: flag to define if slow stop will be applied at end of travel
            default is true
    Kp: Kp PID parameter, default 0.4
    Ki: Ki PID parameter, default 0.005
    Kd: Kd PID parameter, default 0.8
    '''
    global motor_pair

    count = 0
    count_temp = 0
    total_dist = 0
    degree_temp = 0
    degree_pre = 0
    a_Error = 0
    Pre_Error = 0
    RightMotor.set_degrees_counted(0)
    MaxSpeed = math.floor(speed * 5)

    degree_abs = degree_abs % 360
    Yaw = MotionSensor.get_yaw_angle()
    # Convert input from 360 scale to +/-180 scale
    if degree_abs > 179:
        degree_abs = degree_abs - 360
    else:
        degree_abs = degree_abs

    # Calculate Error between two +/-180 scale values
    Error = degree_abs - Yaw
    if Error > 180:
        Error = Error - 360
    elif Error < -180:
        Error = Error + 360

    if dist < 0:
        motor_pair = MotorPair(RightMotorSym, LeftMotorSym)
    else:
        motor_pair = MotorPair(LeftMotorSym, RightMotorSym)
    motor_pair.set_stop_action('hold')

    while total_dist < abs(dist):
        a_Error = a_Error * 0.75 + Error
        d_Error = Error - Pre_Error
        Pre_Error = Error
        C_P = Kp * Error
        C_I = Ki * a_Error
        C_D = Kd * d_Error
        C_Turn = C_P + C_I + C_D
        C_Turn = C_Turn * 0.5
        #C_Turn = C_Turn * 1
        if C_Turn > 0:
            C_Speed_Offset = math.ceil(C_Turn)
            C_Speed_Offset_slow = math.ceil(C_Turn*0.5)
        else:
            C_Speed_Offset = math.floor(C_Turn)
            C_Speed_Offset_slow = math.floor(C_Turn*0.5)

        # Slow Start
        if total_dist <= 0.5:
            motor_pair.start_tank(20 + C_Speed_Offset_slow, 20 - C_Speed_Offset_slow)
        else:
            if slow_stop == 1:
                if abs(dist) - total_dist > 2:
                    #motor_pair.start_tank(min(speed + C_Speed_Offset, MaxSpeed), speed)
                    motor_pair.start_tank(speed + C_Speed_Offset, speed - C_Speed_Offset)
                else:
                    motor_pair.start_tank(15 + C_Speed_Offset_slow, 15 - C_Speed_Offset_slow)
            else:
                motor_pair.start_tank(speed + C_Speed_Offset, speed - C_Speed_Offset)

        # in case robot got stuck
        degree_temp = abs(RightMotor.get_degrees_counted())
        total_dist = degree_temp * ANGLE2DIS
        if count % 10 == 0 and total_dist > 2:
            if degree_temp - degree_pre < 1 :
                count_temp = count_temp + 1
                degree_pre = degree_temp
            else:
                degree_pre = degree_temp

        if count_temp > 3:
            break

        count = count + 1
        Yaw = MotionSensor.get_yaw_angle()
        Error = degree_abs - Yaw
        if Error > 180:
            Error = Error - 360
        elif Error < -180:
            Error = Error + 360

    motor_pair.stop()
    motor_pair = MotorPair(LeftMotorSym, RightMotorSym)

def TurningPID_abs(degree_abs, Kp=2, Ki=0.01, Kd=5, MinPower=20, MaxPower=35):
    '''
    Turning with PID using power

    Parameters:
    degree_abs: absolute degree
    Kp: PID Kp parameter, default 0.8
    Ki: PID Ki parameter, default 0.01
    Kd: PID Kd parameter, default 2
    MinPower: minimal power applied when turning, default 20
    MaxPower: maximal power applied when turning, default 30
    '''
    global motor_pair

    motor_pair = MotorPair(LeftMotorSym, RightMotorSym)
    count = 0
    a_Error = 0
    Pre_Error = 0

    # setting based on experiment
    allowance = 5
    hit_target, Error = on_target(degree_abs, allowance)

    if abs(math.floor(Error + 1)) <= 70:
        allowance = abs(math.floor((Error + 1) / 10))
    else:
        allowance = 5

    motor_pair.set_stop_action("hold")
    motor_pair.stop()

    while True:
        hit_target, Error = on_target(degree_abs, allowance)
        a_Error = (a_Error + Error)
        d_Error = Error - Pre_Error
        Pre_Error = Error
        C_P = Kp * Error
        C_I = Ki * a_Error
        C_D = Kd * d_Error
        C_Turn = C_P + C_I + C_D
        count = count + 1

        if (not hit_target):
            steering = 100 if Error >= 0 else -100
            C_Turn = math.floor(abs(C_Turn))
            C_Turn = min(MaxPower, C_Turn)
            power = min(max(math.floor(C_Turn), MinPower), MaxPower)
            motor_pair.start_at_power(power, steering)
        else:
            motor_pair.stop()
            break

# Turn with both wheels
def Turn(degree):
    TurningPID_abs(degree)
    wait_for_seconds(0.3)
    TurningPID_abs(degree)

def TurningPID_l(degree_abs, Kp=2, Ki=0.01, Kd=5, MinPower=25, MaxPower=35):
    global a_Error, Pre_Error, motor_pair
    RightMotor.set_stop_action("brake")
    RightMotor.stop()
    count = 0
    flag = 0
    a_Error = 0
    Pre_Error = 0
 
    allowance = 2
    hit_target, Error = on_target(degree_abs, allowance)
    #redefine allowance based on the angle (or Error) robot need to make
    if abs(math.floor(Error + 5)) <= 70:
       allowance = abs(math.floor((Error + 5) / 10) )
    else:
       allowance = 2

    while True:        
        hit_target, Error = on_target(degree_abs, allowance)
        a_Error = a_Error + Error
        d_Error = Error - Pre_Error
        Pre_Error = Error
        C_P = Kp * Error
        C_I = Ki * a_Error
        C_D = Kd * d_Error
        C_Turn = C_P + C_I + C_D
        count = count + 1
       
        if (not hit_target):
            #steering = 100 if Error >= 0 else -100
            C_Turn = abs(C_Turn)
            C_Turn = min(MaxPower, C_Turn)
            power = min(max(math.floor(C_Turn), MinPower), MaxPower)
            #print("start_at_power at power", power, ":", Error)
            #motor_pair.start_at_power(power, steering)
            if (Error >=0):
                LeftMotor.start_at_power(-power)
            else:
                LeftMotor.start_at_power(power)
            #motor_pair.start_tank_at_power(max(math.floor(C_Turn), MinPower), -max(math.floor(C_Turn), MinPower))
        else:
            motor_pair.stop()
            break

def Turn_l(degree):
    TurningPID_l(degree)
    wait_for_seconds(0.3)
    TurningPID_l(degree)

def TurningPID_r(degree_abs, Kp=2, Ki=0.01, Kd=5, MinPower=25, MaxPower=35):
    global a_Error, Pre_Error, motor_pair
    LeftMotor.set_stop_action("brake")
    LeftMotor.stop()
    count = 0
    flag = 0
    a_Error = 0
    Pre_Error = 0
 
    allowance = 2
    hit_target, Error = on_target(degree_abs, allowance)
    #redefine allowance based on the angle (or Error) robot need to make
    if abs(math.floor(Error + 1)) <= 70:
       allowance = abs(math.floor((Error + 1) / 10) )
    else:
       allowance = 2

    while True:        
        hit_target, Error = on_target(degree_abs, allowance)
        a_Error = a_Error + Error
        d_Error = Error - Pre_Error
        Pre_Error = Error
        C_P = Kp * Error
        C_I = Ki * a_Error
        C_D = Kd * d_Error
        C_Turn = C_P + C_I + C_D
        count = count + 1
       
        if (not hit_target):
            #steering = 100 if Error >= 0 else -100
            C_Turn = abs(C_Turn)
            C_Turn = min(MaxPower, C_Turn)
            power = min(max(math.floor(C_Turn), MinPower), MaxPower)
            #print("start_at_power at power", power, ":", Error)
            #motor_pair.start_at_power(power, steering)
            if (Error >=0):
                RightMotor.start_at_power(-power)
            else:
                RightMotor.start_at_power(power)
            #motor_pair.start_tank_at_power(max(math.floor(C_Turn), MinPower), -max(math.floor(C_Turn), MinPower))
        else:
            motor_pair.stop()
            break

def Turn_r(degree):
    TurningPID_r(degree)
    wait_for_seconds(0.3)
    #print(MotionSensor.get_yaw_angle())
    TurningPID_r(degree)
    #wait_for_seconds(0.3)
    #print(MotionSensor.get_yaw_angle())
    
def Run2line(whichcolorsensor='Right', speed=30, direction='Forward', line_color='black'):
    '''
    Run robot until hitting line on line_color.

    Parameters:
    whichcolorsensor: defines color sensor to be used
            "Left" - use left color sensor
            Others - use right color sensor, Default
    speed: speed to travel, 0-100. Default 30
    direction: direction to travel
            'Backward'- move backward
            Others    - move forward, Default
    line_color: defines the color to seek, Default 'black'
    '''
    if direction == "Backward":
        motor_pair = MotorPair(RightMotorSym, LeftMotorSym)
    else:
        motor_pair = MotorPair(LeftMotorSym, RightMotorSym)

    if whichcolorsensor == "Left":
        Color = ColorSensor(LeftColorSym)
    else:
        Color = ColorSensor(RightColorSym)

    while True:
        Sensor_color = Color.get_color()
        if Sensor_color == line_color:
            motor_pair.stop()
            break
        else:
            motor_pair.start_tank(speed, speed)

    motor_pair = MotorPair(LeftMotorSym, RightMotorSym)

def Linesquaring(targetcolor='Black', targetintensity=50, int_range=5, direction='Forward'):
    '''
    Use both Color Sensors and Gyro Sensors to squre an edge towards
    "targetcolor", either from White to Black or Black to White with PID,
    in other words, adjustment is proportional to the light reading delta
    Here is the flow of the code:
    1) robot will start moving forward/backward defined by "direction"
    2) Coarse Tuning is performed by making both sensor land on the black
       or white line so that roughly perpendicular to the line
    3) Fine Turning is performed with both color sensors to reach
       "targetintensity" within the margin of error "int_range"
    Measured WHITE and BLACK light intensity on the mat

    Parameters
    targetcolor: defines color starting squaring
        'White' - white color
        Others  - black color, Default
    targetintensity: defines color intensity target
        Default: 30
    int_range: margin of error of color intensity
        Default: 5
    direction: direction of movement
        'Backward' - move backward
        Others     - move forward, Default

    '''
    Left_motor_status = 1
    Right_motor_status = 1
    Left_int = 0
    Right_int = 0
    Kp = 0.25
    if direction == 'Backward':
        motor_pair = MotorPair(RightMotorSym, LeftMotorSym)
        LeftColor = ColorSensor(RightColorSym)
        RightColor = ColorSensor(LeftColorSym)
    else :
        motor_pair = MotorPair(LeftMotorSym, RightMotorSym)
        LeftColor = ColorSensor(LeftColorSym)
        RightColor = ColorSensor(RightColorSym)

    motor_pair.set_stop_action('hold')
    motor_pair.start_tank(20, 20)

    Min = targetintensity - int_range
    Max = targetintensity + int_range
    if targetcolor == 'White':
        while True:
            Left_color = LeftColor.get_color()
            Right_color = RightColor.get_color()
            if Left_color == 'black' and Right_color == 'black':
                motor_pair.stop()
                break
            elif Left_color == 'black':
                Left_motor_status = 0
                if Right_motor_status == 0:
                    motor_pair.stop()
                    break
                else:
                    motor_pair.start_tank(-3, 15)
            elif Right_color == 'black':
                Right_motor_status = 0
                if Left_motor_status == 0:
                    motor_pair.stop()
                    break
                else:
                    motor_pair.start_tank(15, -3)

        # Fine Tuning
        Left_motor_status = 1
        Right_motor_status = 1

        while True:
            Left_int = 100 * (LeftColor. get_reflected_light() - BLACK) / (WHITE - BLACK)
            Right_int = 100 * (RightColor. get_reflected_light() - BLACK) / (WHITE - BLACK)
            # Adjustment proportional to the intensity delta for each wheel,
            # and prevent it to be too small
            Lspeed = min(max(math.floor(abs(Left_int - targetintensity) * Kp), 10), 25)
            Rspeed = min(max(math.floor(abs(Right_int - targetintensity) *Kp), 10), 25)
            if Min <= Left_int <= Max and Min <= Right_int <= Max:
                motor_pair.stop()
                break
            elif Min <= Left_int <= Max:
                Left_motor_status = 0
                if Right_motor_status == 0:
                    motor_pair.stop()
                    break
            elif Min <= Right_int <= Max:
                Right_motor_status = 0
                if Left_motor_status == 0:
                    motor_pair.stop()
                    break
            elif Left_int < Min and Right_int > Max:
                motor_pair.start_tank(Lspeed, -Rspeed)
            elif Left_int > Max and Right_int < Min:
                motor_pair.start_tank(-Lspeed, Rspeed)
            elif Left_int < Min and Right_int < Min:
                motor_pair.start_tank(Lspeed, Rspeed)
            elif Left_int > Max and Right_int > Max:
                motor_pair.start_tank(-Lspeed, -Rspeed)
    """ else:
        while True:
            Left_color = LeftColor.get_color()
            Right_color = RightColor.get_color()
            if Left_color == 'white' and Right_color == 'white':
                motor_pair.stop()
                break
            elif Left_color == 'white':
                Left_motor_status = 0
                if Right_motor_status == 0:
                    motor_pair.stop()
                    break
                else:
                    motor_pair.start_tank(-3, 15)
            elif Right_color == 'white':
                Right_motor_status = 0
                if Left_motor_status == 0:
                    motor_pair.stop()
                    break
                else:
                    motor_pair.start_tank(15, -3)

        # Fine Tuning
        Left_motor_status = 1
        Right_motor_status = 1
        while True:
            Left_int = 100 * (LeftColor. get_reflected_light() - BLACK) / (WHITE - BLACK)
            Right_int = 100 * (RightColor. get_reflected_light() - BLACK) / (WHITE - BLACK)
            # Adjustment proportional to the intensity delta for each wheel,
            # and prevent it to be too small
            Lspeed = min(max(math.floor(abs(Left_int - targetintensity) * Kp), 10), 25)
            Rspeed = min(max(math.floor(abs(Right_int - targetintensity) *Kp), 10), 25)
            if Min <= Left_int <= Max and Min <= Right_int <= Max:
                motor_pair.stop()
                break
            elif Min <= Left_int <= Max:
                Left_motor_status = 0
                if Right_motor_status == 0:
                    motor_pair.stop()
                    break
            elif Min <= Right_int <= Max:
                Right_motor_status = 0
                if Left_motor_status == 0:
                    motor_pair.stop()
                    break
            elif Left_int < Min and Right_int > Max:
                motor_pair.start_tank(-Lspeed, Rspeed)
            elif Left_int > Max and Right_int < Min:
                motor_pair.start_tank(Lspeed, -Rspeed)
            elif Left_int < Min and Right_int < Min:
                motor_pair.start_tank(-Lspeed, -Rspeed)
            elif Left_int > Max and Right_int > Max:
                motor_pair.start_tank(Lspeed, Rspeed) """

    motor_pair = MotorPair(LeftMotorSym, RightMotorSym)

""" def trip_1():
    StraightPID_right(0, 20, 35)
    Turn_l(15)
    StraightPID_right(15, 14, 35)
    Turn_r(-25)
    StraightPID_right(-25, 2, 30, slow_stop=0)
    Turn_l(20)
    StraightPID_right(20, 20, 35)
    Turn_l(45)
    StraightPID_right(45, 21, 35)
    Turn(90)
    StraightPID_right(90, 14, 35)
    Turn_l(112)
    StraightPID_right(112, 16, 35)
    Run2line('Right', 30, 'Forward', 'black')
    StraightPID_right(112, 5.5, 35)
    Turn_l(-178)
    StraightPID_right(-178, 30, 35)
    #Turn(-178)
    StraightPID_right(-178, 13.8, 50, slow_stop=0)
    StraightPID_right(-178, -4, 40)
    Turn(-115)
    StraightPID_right(-115, 70, 80)
    
def trip_2():
    #Navivagting to Hydroelectric Dam
    StraightPID_right(0, 75, 30)
    Run2line("Right", 30, "Forward", "black")
    Turn(-90)
    #wait_for_seconds(0.5)
    StraightPID_right(-90, 23, 30)
    Turn(136)
    #wait_for_seconds(0.5)
    
    #Hydroelectric Dam
    motor_pair.move_tank(2, "seconds", 15, 15)
    wait_for_seconds(0.3)
    
    #Back to Base 
    StraightPID_right(142, -20, 35)
    Turn_l(120)
    StraightPID_right(120, 35, 50)
    Turn_l(160)
    StraightPID_left(160, 70, 50)
    # Turn_l(90)
    # StraightPID_right(90, 33, 40)
    # Turn_l(180)
    # StraightPID_left(180, 70, 80)

def trip_3():
    #Push the television and go back
    # motor_pair.move_tank(2.5, "seconds", 30, 30)
    # motor_pair.move_tank(5, "cm", -30, -30)
    StraightPID_right(0, 31, 35)

    #Navigate to windmill
    Turn(-45)
    StraightPID_double(-45, 39, 40)
    Turn(43)

    #Windmill mission
    i = 1
    while i<4:
        #motor_pair.move_tank(25, "cm", 60, 60)
        StraightPID_double(43, 11, 50, slow_stop=0)
        wait_for_seconds(0.3)
        StraightPID_double(43, -8, 40)
        wait_for_seconds(0.3)
        i = i+1

    #Navigating to Hybrid Car & doing Rechargable Battery
    motor_pair.move_tank(1.5, "seconds", -45, -45)
    StraightPID_double(43, 1, 45)
    
    Turn(-44)
    StraightPID_right(-44, 32, 30) 
    #Turn(-43)
    #StraightPID_right(-43, 15.5, 28)
    RightArm.run_for_degrees(-250, 100)
    wait_for_seconds(1)
    StraightPID_left(-43, -105, 50)   
    
def trip_4():
    StraightPID_double(0, 48, 40)
    wait_for_seconds(1)
    motor_pair.move_tank(2.5, "seconds", -40, -40)

def trip_5():
    # push innovation project
    StraightPID_right(0, 72, 40)
    Turn(45)
    StraightPID_right(45, 35, 40)
    StraightPID_right(45, -7, 40)
    # Hand
    Turn(90)
    
    StraightPID_double(90, 36, 60)
    StraightPID_double(90, -10, 40)
    
    # square
    Run2line()
    #wait_for_seconds(1)
    #Linesquaring("White", 50, 4, "Forward")
    # solar farm
    StraightPID_right(90, -12, 40)
    Turn_r(31)
    StraightPID_right(31, 38, 40)
    Turn_l(0)
    StraightPID_right(0, -10, 40)
    wait_for_seconds(0.2)
    LeftArm.run_for_rotations(0.6, -100)
    Turn(-2)
    StraightPID_double(-2, 27, 40)
    LeftArm.run_for_rotations(0.6, 100)
    Turn_l(-10)
    StraightPID_right(-10, -3, 30)
    Turn(-55)
    StraightPID_right(-55, 80, 70) 

def trip_6():
    hitangle = -15
    
    StraightPID_double(0, 55, 40)
    Turn(hitangle)
    
    StraightPID_left(hitangle, 25, 40, slow_stop=0)
    wait_for_seconds(0.5)
    StraightPID_double(hitangle, -4, 30)
    wait_for_seconds(0.5)
    for i in range(3):
        StraightPID_left(hitangle, 5, 50, slow_stop=0)
        wait_for_seconds(0.5)
        StraightPID_double(hitangle, -4, 30, slow_stop=0)
        wait_for_seconds(0.5)

    
    StraightPID_double(hitangle+5, -35, 40)
    StraightPID_double(0, -50, 50)"""


    
# def trip_2():
#     # Toy factory
#     StraightPID_right(0, -41, 40)
#     wait_for_seconds(0.5)
#     StraightPID_double(0, 13, 40)
#     Turn(135) 
#     StraightPID_right(135, 41, 40)
#     # Find black line and get ready do Hand
#     Run2line('Left', 30, 'Forward', 'black')
#     wait_for_seconds(0.3)
#     Linesquaring('White', 30, 5, 'Forward')
#     wait_for_seconds(0.3)
#     MotionSensor.reset_yaw_angle()
#     #wait_for_seconds(0.3)
    
#     StraightPID_right(0, 8, 40)
#     Turn(90)
#     StraightPID_right(90, 30, 40)
#     StraightPID_right(90, 10, 35)
#     # #wait_for_seconds(0.3)
#     StraightPID_right(90, -38, 40)
#     Turn(60)
#     StraightPID_right(60, 15, 35)
#     Turn(80)
#     StraightPID_right(80, 32, 40)
#     Turn_r(22)
#     StraightPID_right(22, 14, 35)
#     Turn_r(-50)
#     StraightPID_right(-50, 65, 40)
#     StraightPID_right(-50, -16, 40)
#     Turn_l(-120)
#     StraightPID_right(-120, 12, 35)
#     Turn(-45)
#     StraightPID_right(-45, 40, 40)

def trip_3():
    # Water reservoir
    StraightPID_right(0, -10, 40)
    Turn_l(-10)
    StraightPID_right(-10, -15, 35)
    
    Turn_l(-25)
    StraightPID_right(-25, -15, 35)
    Turn_l(-47)
    StraightPID_right(-47, -45, 35)
    
    Turn_l(-20)
    StraightPID_right(-20, 12, 35)
    # Turn_r(-20)
    # StraightPID_right(-20, 3, 35)
    Turn_r(-32)
    StraightPID_right(-32, 8, 30)
    #motor_pair.move_tank(2, "seconds", 15, 15)
    wait_for_seconds(1)
    
    
    StraightPID_right(-32, -22, 30)
    Turn(-90)
    StraightPID_right(-90, 20, 40)
    Turn(-90)
    StraightPID_double(-90, 10, 40, slow_stop=0)
    wait_for_seconds(0.5)
    StraightPID_right(-90, -5, 35)
    Turn(0)
    StraightPID_right(0, 50, 40)
    # Run2line('Left', 30, 'Forward', 'black')
    
    
    # Run2line('Left', 30, 'Forward', 'black')
    # StraightPID_right(20, 8, 40)
    # Turn(90)
    # StraightPID_right(90, 38, 40)
    # #wait_for_seconds(0.3)
    # StraightPID_right(90, -40, 40)
    # Turn(58)
    # StraightPID_right(58, 16, 35)
    # Turn(80)
    # StraightPID_right(80, 35, 40)
    # Turn_r(22)
    # StraightPID_double(22, 15, 35)
    # Turn_r(-50)
    # StraightPID_right(-50, 65, 40)
    # StraightPID_right(-50, -18, 40)
    # Turn_l(-125)
    # StraightPID_right(-125, 5, 35)
    # Turn(-45)
    # StraightPID_right(-45, 30, 40) 

# wait()
# initialize()
# trip_inprogress()
# trip_2()
# done()  
  
wait()
initialize()
trip_inprogress()
trip_3()
done()  
    



# wait()
# initialize()
# trip_inprogress()
# trip_4()
# done()

# wait()
# initialize()
# trip_inprogress()
# trip_5()
# done()

# wait()
# initialize()
# trip_inprogress()
# trip_6()
# done()