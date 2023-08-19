#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor,TouchSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
ev3.speaker.beep()

SPEED = 100
EPSILON = 2
# Initialize motors at port A and D
right_motor = Motor(Port.A)
left_motor = Motor(Port.D)

# Initialize gyro sensors
gyro_sensor =  GyroSensor(Port.S2)
curr_angle = 0
prev_angle = 0
gyro_start_angle = gyro_sensor.angle()
# Initialize touch sensors
touch_sensor =  TouchSensor(Port.S3)
switch_state = "OFF"
press_duration = 0
is_switch_disable = False
# infrared_sensor = InfraredSensor(Port.S1)

# Initialize the timers.
action_timer = StopWatch()


def set_speed(speed_left=0, speed_right=0):
    left_motor.dc(speed_left)
    right_motor.dc(speed_right)


leave_the_stand_iterations = 150
def wake_up():
    ev3.screen.load_image(ImageFile.AWAKE)
    ev3.light.on(Color.GREEN)
    global gyro_angle
    global leave_the_stand_iterations
    global gyro_start_angle
    while leave_the_stand_iterations > 0:
        set_speed(50,50)
        gyro_angle = 0
        gyro_start_angle = gyro_sensor.angle()
        if leave_the_stand_iterations == 1:
            set_speed(0,0)
        leave_the_stand_iterations -= 1
    start()



def stop():
    ev3.light.off()
    ev3.screen.load_image(ImageFile.SLEEPING)
    global leave_the_stand_iterations
    # Reset the sensors and variables.
    leave_the_stand_iterations = 150
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    set_speed(0,0)

def start():
    # wake up 
    # ev3.speaker.play_file(SoundFile.SPEED_UP)
    global curr_angle
    global prev_angle
    global gyro_start_angle
    # gyro_sensor_speed = gyro_sensor.speed()
    prev_angle = curr_angle
    curr_angle = gyro_sensor.angle()
    delta = abs(curr_angle - gyro_start_angle)

    speed = delta*10 + 20
    speed = max(40, speed)
    if curr_angle > gyro_start_angle + 2:
        set_speed(speed,speed)
    elif curr_angle < gyro_start_angle :
        set_speed(-speed,-speed)
    else:
        set_speed(0,0)




wait(500)
while 1:
    is_switch_disable = False
    while touch_sensor.pressed():
        if not is_switch_disable:
            if switch_state == "ON":
                switch_state="OFF"
                is_switch_disable=True
            elif switch_state == "OFF":
                switch_state="ON"
                is_switch_disable=True



    if switch_state == "ON":
        wake_up()  
    else:
        stop()



        

   
        









