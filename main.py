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
switch_state = "ON"
press_duration = 0
is_switch_disable = False
# infrared_sensor = InfraredSensor(Port.S1)

# Initialize the timers.
action_timer = StopWatch()


def set_speed(speed_left=0, speed_right=0):
    left_motor.dc(speed_left)
    right_motor.dc(speed_right)

def hold():
    # left_motor.dc(0)
    left_motor.hold()
    # right_motor.dc(0)
    right_motor.hold()


leave_the_stand_iterations = 100
def wake_up():
    global leave_the_stand_iterations
    global gyro_start_angle
    ev3.screen.load_image(ImageFile.SLEEPING)
    wait(1000)
    ev3.speaker.play_file(SoundFile.SPEED_UP)
    ev3.screen.load_image(ImageFile.AWAKE)
    ev3.light.on(Color.GREEN)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    gyro_start_angle = gyro_sensor.angle()
    
    while leave_the_stand_iterations > 0:
        set_speed(30,30)
        gyro_start_angle = gyro_sensor.angle()
        if leave_the_stand_iterations == 1:
            set_speed(0,0)
        leave_the_stand_iterations -= 1



def stop():
    ev3.light.off()
    ev3.screen.load_image(ImageFile.SLEEPING)
    global leave_the_stand_iterations
    # Reset the sensors and variables.
    leave_the_stand_iterations = 100
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    set_speed(0,0)

def start():
    # wake up 
    # ev3.speaker.play_file(SoundFile.SPEED_UP)
    global curr_angle
    global prev_angle
    global gyro_start_angle
    curr_angle = gyro_sensor.angle()
    delta = abs(curr_angle - gyro_start_angle)

    speed = delta*8
    speed = max(50, speed)
    if curr_angle > gyro_start_angle:
        set_speed(speed,speed)
    elif curr_angle < gyro_start_angle :
        set_speed(-speed,-speed)
    else:
        hold()



# wake_up()
# left_motor.run_time(50,3000)

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
        while 1 :
            start()
            while touch_sensor.pressed():
                switch_state = "OFF"
            if  switch_state == "OFF":
                break
    else:
        stop()



        

   
        









