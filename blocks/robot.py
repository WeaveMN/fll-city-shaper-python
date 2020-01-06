from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

# Custom robot class containing our functions that leverage the Gyro sensor
class Robot:

  def __init__(self, left_motor_port, right_motor_port, med_motor_port, gyro_port, wheel_diameter, wheel_base):
    self.left_motor = Motor(left_motor_port)    
    self.right_motor = Motor(right_motor_port)
    # self.med_motor = Motor(med_motor_port)
    self.robot = DriveBase(self.left_motor, self.right_motor, wheel_diameter, wheel_base)
    self.gyro = GyroSensor(gyro_port)

  # Function definitions

  # Gyro sensor reset, waits until the gyro has come to rest and then resets the value to zero
  # use at beginning of mission programs
  def resetGyro(self):
    brick.light(Color.RED)
    speed = self.gyro.speed()
    angle = self.gyro.angle()
    while speed != 0:
      wait(100)
      speed = self.gyro.speed()
      angle = self.gyro.angle()
    self.gyro.reset_angle(0)
    brick.light(Color.GREEN)

  # Drive the robot straight using the GyroSensor
  # 
  def driveStraight(self, rotations, speed):
    distance = rotations * 360 # convert wheel rotations to degrees
    self.gyro.reset_angle(0)
    self.left_motor.reset_angle(0)
    self.right_motor.reset_angle(0)
    # set our amount to correct back towards straight
    correction = -2
    if speed < 0:
      correction = 2
    # start the robot driving
    self.robot.drive(speed, 0)
    # loop until the robot has travelled the distance we want
    # updating the steering angle of the drive based on the gyro measured drift and correction
    while abs(self.left_motor.angle()) <= distance and abs(self.right_motor.angle()) <= distance:
      drift = self.gyro.angle()
      print("Drift: " + str(drift))
      steering = drift * correction
      #print("Steering: " + str(steering))
      self.robot.drive(speed, steering)
    self.robot.stop(Stop.BRAKE)

  #  Turn the robot an exact amount using the GryoSensor
  def turnDegrees(self, degrees):
    self.gyro.reset_angle(0)
    initial_power = 300
    end_power = 50
    left_motor_power = initial_power
    right_motor_power = initial_power * -1
    if degrees < 0:
      left_motor_power = initial_power * -1
      right_motor_power = initial_power
    initial_turn = abs(degrees * .75)
    self.left_motor.run(left_motor_power)
    self.right_motor.run(right_motor_power)
    angle = self.gyro.angle()
    print("Angle: " + str(angle))
    while abs(angle) < initial_turn:
      wait(10)
      angle = self.gyro.angle()
      print("Angle: " + str(angle))
    left_motor_power = end_power
    right_motor_power = end_power * -1
    if degrees < 0:
      left_motor_power = end_power * -1
      right_motor_power = end_power
    self.left_motor.run(left_motor_power)
    self.right_motor.run(right_motor_power)
    end_degrees = (abs(degrees) -1)
    angle = self.gyro.angle()
    print("Angle: " + str(angle)) 
    while abs(angle) < end_degrees:
      wait(10)
      angle = self.gyro.angle()
      print("Angle: " + str(angle))
    self.left_motor.stop(Stop.BRAKE)
    self.right_motor.stop(Stop.BRAKE)
    print("Final Angle: " + str(self.gyro.angle()))
  
  def oldDrive(self, speed, turn):
    while True:
      self.robot.drive(speed, turn)