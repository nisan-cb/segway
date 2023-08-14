#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
ev3.speaker.beep()

SPEED = 50
EPSILON = 5
# Initialize motors at port A and D
right_motor = Motor(Port.A)
left_motor = Motor(Port.D)

# Initialize gyro and infrared sensors
gyro_sensor =  GyroSensor(Port.S2)
# infrared_sensor = InfraredSensor(Port.S3)

# Initialize gyro and infrared sensors

def set_speed(speed_left=0, speed_right=0):
    left_motor.dc(speed_left)
    right_motor.dc(speed_right)


wait(1000)

set_speed(50,50)
wait(500)


while 1:

    gyro_sensor_value = gyro_sensor.speed()
    ev3.screen.draw_text(40, 50, gyro_sensor_value )
    ev3.screen.clear()
    if gyro_sensor_value > EPSILON :
        set_speed(SPEED,SPEED)
    elif gyro_sensor_value < -EPSILON :
        set_speed(-SPEED,-SPEED)
    else:
        set_speed(0,0)
        









