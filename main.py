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
EPSILON = 5
# Initialize motors at port A and D
right_motor = Motor(Port.A)
left_motor = Motor(Port.D)

# Initialize gyro sensors
gyro_sensor =  GyroSensor(Port.S2)
gyro_angle = 0
# Initialize touch sensors
touch_sensor =  TouchSensor(Port.S3)
switch_state = "OFF"
press_duration = 0
is_switch_disable = False
# infrared_sensor = InfraredSensor(Port.S1)

# Initialize gyro and infrared sensors

def set_speed(speed_left=0, speed_right=0):
    left_motor.dc(speed_left)
    right_motor.dc(speed_right)

def stop():
    set_speed(0,0)

def start():
    global gyro_angle
    gyro_sensor_value = gyro_sensor.speed()
    # ev3.screen.draw_text(40, 50, gyro_sensor_value )
    # ev3.screen.clear()
    gyro_angle += gyro_sensor_value
    if gyro_angle > EPSILON :
        set_speed(SPEED,SPEED)
    elif gyro_angle < -EPSILON :
        set_speed(-SPEED,-SPEED)
    else:
        set_speed(0,0)


wait(500)


while 1:
    if not is_switch_disable and touch_sensor.pressed():
        press_duration += 1
        if press_duration>25 and switch_state == "ON":
            switch_state="OFF"
            ev3.screen.clear()
            is_switch_disable=True
        elif press_duration>25 and switch_state == "OFF":
            ev3.screen.clear()
            switch_state="ON"
            is_switch_disable=True
    elif not touch_sensor.pressed():
        press_duration=0
        is_switch_disable=False


    ev3.screen.draw_text(40, 50,  switch_state)

    if switch_state == "ON":
        start()
    else:
        stop()


        

   
        









