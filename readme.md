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
grab_motor = Motor(Port.A)
shooting_motor=Motor(Port.D)

# Write your program here.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=115)

ev3.speaker.beep()

shooting_motor.run_until_stalled(-100,Stop.COAST, duty_limit=50)
grab_motor.run_until_stalled(100,Stop.COAST,duty_limit=50)
grab_motor.reset_angle(0)

grab_motor.run_tarhet(100,-100)

robot.straight(100)

grab_motor.run_until_stalled(200,Stop.COAST,duty_limit=50)

grab_motor.run_until_stalled(-200,Stop.COAST,duty_limit=50)
shooting_motor.run(2000)
time.sleep(0.25)
shooting_motor.stop()