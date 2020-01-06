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
missionList = ["TAN STACK", "RED STACK", "CRANE"]

def displayMenu():
  # Clear display
  
  # Create a menu for the missions
  selectedMission = 0
  length = len(missionList)
  repaint = True

  while True:
    if repaint:
      brick.display.clear()
      brick.display.text("City Shaper Missions:", (0,20))
      for i in range(length):
        if i == selectedMission:
          prefix = ">>> "
        else:
          prefix = "    "
        brick.display.text(prefix + str(i) + ". " + missionList[i] )
      repaint = False  
    up_pressed = Button.UP in brick.buttons()
    down_pressed = Button.DOWN in brick.buttons()
    center_pressed = Button.CENTER in brick.buttons()
    if center_pressed:
      break
    elif up_pressed:
      selectedMission = selectedMission - 1
      if selectedMission < 0:
        selectedMission = 0
      repaint = True
    elif down_pressed:
      selectedMission = selectedMission + 1
      if selectedMission >= length:
        selectedMission = selectedMission - 1
      repaint = True

  return selectedMission

# while True:
#   selected = displayMenu()
#   mission = missionList[selected]
#   print("Running " + mission)






# TEST IT OUT
robot.resetGyro()
robot.driveStraight(4, 360)
robot.turnDegrees(90)
# robot.oldDrive(0, 90)

