#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import UARTDevice
import time


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

robot = DriveBase(left_motor, right_motor, wheel_diameter = 5.6, axle_track = 115)
grab_motor = Motor(Port.D)
shooting_motor = Motor(Port.A)
gyro = GyroSensor(Port.S2)


ser = UARTDevice(Port.S3, baudrate = 115200)

while True:
    data = ser.read_all()
    print(data)
    time.sleep_ms(1000)

def data_filter(data):
    decoded_data = data.decode().strip()
    if decoded_data.isdigit():
        return int(decoded_data)
    return None

def p_control(cam_data,kp,power):
    error = threshold-cam_data
    output = error * kp
    robot.drive(power,output)


def pd_control(cam_data, kp, kd, power):
    global previous_error
    error = threshold - cam_data
    derivative = error - previous_error
    output = (kp * error) + (kd * derivative)
    robot.dirve(power, ourput)
    previous_error = error


ev3.speaker.beep()
threshold = 200

while True:
    data = ser.read_all()
    if data:
        filtered_data = data_filter(data)
        if filtered_data is not None:
            print(filtered_data)
            pd_control(filtered_data, kp = 0.5, kd = 0.1, power = 100)
        wait(10)





# shooting_motor.run_until_stalled(-100, Stop.COAST, duty_limit = 50)
# grab_motor.run_until_stalled(100, then = Stop.COAST, duty_limit = 50)
# grab_motor.reset_angle(0)


# grab_motor.run_target(100,-100)
# robot.straight(100)

# grab_motor.run_until_stalled(200, then = Stop.COAST, duty_limit = 50)
# grab_motor.run_until_stalled(-200, then = Stop.COAST, duty_limit = 50)
# robot.straight(-50)

# while gyro.angle() < 90:
#     pass

# robot.stop()

# shooting_motor.run(2000)
# time.sleep(0.25)
# shooting_motor.stop()

# grab_motor.run_until_stalled(100, then = Stop.COAST, duty_limit = None) #10cm앞 그거
# grab_motor.reset_angle(0)
# robot.straight(100)

# grab_motor.run_until_stalled(-100, then = Stop.COAST, duty_limit = None)
# grab_motor.run_target(100,-100)
# shooting_motor.run_until_stalled(-100, Stop.COAST, duty_limit = 50)

# shooting_motor.run(2000)
# time.sleep(0.25)
# shooting_motor.stop()