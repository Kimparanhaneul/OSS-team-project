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
gyro = GyroSensor(Port.S2)
ser = UARTDevice(Port.S4, baudrate=115200)
touch_sensor = TouchSensor(Port.S1)  # 터치 센서 추가
#==========[motors]==========
shooting_motor = Motor(Port.D)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=115)

#==========[target_angle turn(gyro)]==========
def turn(target_angle, power):
    while True:
        # 현재 각도 읽기
        current_angle = gyro.angle()
        ev3.screen.clear()

        # 목표 각도와 현재 각도의 차이 계산
        angle_difference = target_angle - current_angle
      
        # 회전 방향 및 회전 제어
        turn_power = power if angle_difference > 0 else -power
        robot.drive(0, turn_power)

        # 목표 각도에 근접하면 멈춤
        if abs(angle_difference) < 2:  # 각도 차이가 2도 이내면 정지
            robot.stop()
            break


#==========[camera_chase]==========
def process_uart_data(data):
    try:
        # 데이터를 문자열로 디코드 (키워드 인자 제거)
        data_str = data.decode().strip()
        if not data_str:
            pass

        # 문자열에서 리스트 파싱
        data_str = data_str.strip("[]")
        parsed_list = [int(value.strip()) for value in data_str.split(",")]

        # 파싱된 결과 반환
        return parsed_list
    except:
        # 에러 처리
        return [-1,-1] # -1이 나오면 무시하는 코드 사용

def pd_control(cam_data, kp, kd, power):
    global previous_error
    error = cam_data - threshold
    derivative = error - previous_error
    output = (kp * error) + (kd * derivative)
    robot.drive(power, output)
    previous_error = error

#==========[shooting positions]==========
def grab(command):
    if command == 'motion3':
        #close
        grab_motor.run_until_stalled(1000,Stop.COAST,duty_limit=50)
        #set_zero point
        grab_motor.reset_angle(0)
    elif command == 'motion1':
        #open1
        grab_motor.run_until_stalled(-500,Stop.COAST,duty_limit=50)
    elif command == 'motion2':
        #open2
        grab_motor.run_target(2000,-150) #각도 속도

def shoot(command):
    if command == 'zero':
        #zero_position
        shooting_motor.run_until_stalled(-100,Stop.COAST,duty_limit=50)
    elif command == 'shoot':
        #shooting
        shooting_motor.run(2600)
        time.sleep(0.25)
        shooting_motor.stop()




#==========[setup]==========
ev3.speaker.beep()
threshold = 80
previous_error = 0
gyro.reset_angle(0)
#==========[zero set position setting]==========
shoot('zero') #shoot 모터가 안쪽이고,
grab('motion3') #grab 모터가 바깥쪽이므로 shoot먼저 세팅 후 grab을 세팅해야한다
time.sleep(1)
grab('motion2') #공을 잡기 위한 높이로 열기

print("Zero set postion completed")

#==========[main loop]==========
while True:
    if touch_sensor.pressed():  # 터치 센서가 눌렸는지 확인
        time.sleep(0.1)  # 짧은 시간 대기 (디바운싱 처리)
        if touch_sensor.pressed():  # 눌린 상태가 지속되는지 확인
            robot.straight(-200)  # 20cm 뒤로 이동
            robot.stop()
            time.sleep(0.5)  # 터치 후 잠시 대기
    
    data = ser.read_all()
    # 데이터 처리 및 결과 필터링
    try:
        filter_result = process_uart_data(data)
        #filter_result[0] : x, filter_result[1] : y
        if filter_result[0]!= -1 and filter_result[1]!= -1:
        # if filter_result[0]!= -1 and filter_result[1]!= -1:
            if filter_result[1] > 91: #공이 카메라 화면 기준으로 아래에 위치 = 로봇에 가까워졌다
                robot.straight(40) #강제로 앞으로 이동
                grab('motion3') #공을 잡기
                time.sleep(0.5) #동작간 딜레이
                robot.straight(70)
                time.sleep(0.5) #동작간 딜레이
                robot.straight(-50)
                robot.stop()
               # grab('motion1') #슛을 위한 열기
                #time.sleep(0.2) #동작간 딜레이
                robot.straight(300)
                #turn(30, 100) #정면(상대방 진영)바라보기
                time.sleep(0.5) #동작간 딜레이
                shoot('shoot') #공 날리기
                time.sleep(0.5) #동작간 딜레이
                shoot('zero')
                time.sleep(0.5) #동작간 딜레이
                grab('motion2')
                time.sleep(1) #동작간 딜레이
                robot.straight(200)
 
                #shoot('zero')
                #grab('motion2') 
            else: #공이 카메라 화면 기준 멀리 위치해 있으면 chase한다
                pd_control(filter_result[0], kp=0.5, kd=0.1, power=150) ##로봇속도
      #  else:  # 공 감지 실패
           # robot.straight(100)
       # time.sleep(0.5)
    except:
        pass


