#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import UARTDevice
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

    
#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop
from pybricks.robotics import DriveBase
import time

ev3 = EV3Brick()

#==========[motors]==========
shooting_motor = Motor(Port.D)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=115)

# Set robot's drive speed (faster)
robot.settings(straight_speed=300)  # Speed in mm/s

#==========[shooting function]==========
def shoot():
    # Shooting process
    #shooting_motor.run(1000)  # High speed for shooting
    shooting_motor.dc(100)
    time.sleep(0.25)  # Hold position briefly
    shooting_motor.stop()  # Stop the motor after shooting
    
    # Wait for 0.1 seconds before returning to initial position
    time.sleep(0.25)
    
    # Reset shooting mechanism to initial position
    shooting_motor.run_until_stalled(-300, Stop.COAST, duty_limit=30)
    time.sleep(0.25)

#==========[main loop]==========
while True:
    # Move forward 1 meter faster
    time.sleep(10)
    robot.straight(880)  # Forward movement
    # 현재 설정된 속도 출력
    print(robot.settings())

    
    # Shoot the ball
    shoot()
    
    # Move backward 1 meter faster
    robot.straight(-875)  # Backward movement
    
    # Wait for 1 second
    time.sleep(0.5)


