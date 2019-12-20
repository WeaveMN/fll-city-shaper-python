#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
from blocks.robot import Robot

# Write your program here
brick.sound.beep()

# Global definitions for motors, drive, and sensors 
# left_motor = Motor(Port.D)
# right_motor = Motor(Port.A)
# med_motor = Motor(Port.B)
# robot = DriveBase(left_motor, right_motor, 37 ,95)
# gyro = GyroSensor(Port.S4) #  Check port

# Use our custom robot class
# parameters: (left_motor_port, right_motor_port, med_motor_port, gyro_port, wheel_diameter, wheel_base)
robot = Robot(Port.D, Port.A, Port.B, Port.S4, 55, 120)

# TEST IT OUT
robot.resetGyro()
robot.driveStraight(4, 500)
robot.turnDegrees(90)

