import time
import board
import adafruit_bmp280
import RPi.GPIO as GPIO
import numpy as np
import serial
import datetime
from multiprocessing import Process
import sys
import matplotlib.pyplot as plt
import keyboard
import os
from pyubx2 import UBXReader


def ebimu_process(n):

    ############################## 파일 생성 #################################

    nowTime = str(datetime.datetime.now())
    fileName = nowTime[:10]+"_ebimu_"+nowTime[11:21]
    try:
        f = open(fileName, 'w')
        log = open("Ebimu_log.txt",'w')
    except:
        print("Failed to open file, EBIMU")
        sys.exit()
        
    ############################## 센서 연결 #################################

    ser = serial.Serial('/dev/ttyUSB0',115200,timeout=0.001)

    ############################ 데이터 수집 #################################

    buf = "" 
    while True:
        while ser.inWaiting():
            data = str(ser.read()).strip() 
            buf += data
            if data[3] == "n":
                buf = buf.replace("'","")
                buf = buf.replace("b","") 
            
                try : 
                    roll, pitch, yaw, x, y, z = map(float,buf[1:-4].split(','))
                except Exception as e:
                    #print(buf)
                    print("Error from data processing : ", e)
                    log.write("Error from data processing : "+str(e)+"\n With '"+buf+"'\n")
                    buf = ""
                    continue
                
                datas = [roll,pitch,yaw,x,y,z]
                writeString = "*"+str(datas)[1:-1]+"\n"
                f.write(writeString)
                
                print(roll,pitch,yaw,x,y,z)
                buf = ""

if __name__ == '__main__':

    ################################ 초기 화면 ###################################

    title = "GOAT Rocket Program"
    for i in range(len(title)):
        os.system('clear')
        print("+-----------------------+")
        print('|  ',end="")
        print(title[:i+1], end='')
        if i != len(title)-1: print('|',end='')
        print((' '*(17-i))+'  |')
        print("+-----------------------+")
        time.sleep(0.1)

    ############################## 부품 연결과 설정 #################################

    stream = serial.Serial("/dev/ttyACM0", baudrate=19200, timeout=1)
    ubr = UBXReader(stream)

    gf =  open("gps.txt", "a")


    # 통신 모듈 연결
    ser = serial.Serial(
        port="/dev/ttyAMA0",
        baudrate=19200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )

    #Bmp280 센서 연결
    i2c = board.I2C()  
    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)
    # 표준 대기압으로 설정
    bmp280.sea_level_pressure = 1013.25

    #서보 모터 설정
    GPIO.setmode(GPIO.BCM)
    servo_pin = 18
    GPIO.setwarnings(False)
    GPIO.setup(servo_pin, GPIO.OUT)
    pwm = GPIO.PWM(servo_pin, 50)
    pwm.start(0) 

    time.sleep(0.5)
    print("Components Connectiont OK")

    ##################################
    # 서보 동작 테스트 코드 추가 공간
    ##################################

    ################################# 자료 선언 ###################################

    # 상태 딕셔너리
    status = {
        "parachute": "0",  # 예시 데이터, Not yet deploy:사출 전, Deploy:사출 후, Force Deploy:강제 사출 후
        "way": "DOWN",  # 예시 데이터, UP:상승시, DOWN(3):하강시 괄호 안은 카운트
        "ebimu": "-",  # 예시 데이터, 120,512,252: x,y,z
        "bmp": "0",  # 예시 데이터,  50: 고도
    }

    WINDOW = 20
    NO_DEPLOY_ALTITUDE = 1
    FALLING_CONFIRMATION = 10
    ESTIMATED_MAX_ALTITUDE = 400

    datas = [0]
    moving_averages = []
    falling_count = 0
    is_deployed = False
    is_outlier = False
    is_valid_falling = False
    
    ################################# 파일 생성 ###################################

    now = str(datetime.datetime.now())
    fileN = now[:10]+"_bmp_"+now[11:21]
    fileS = now[:10]+"_servoLog_"+now[11:21]
    f = open(fileN, 'w')
    s = open(fileS, 'w')
    time.sleep(0.5)
    print("File creation OK")

    ################################ 고도 영점 연산 #################################

    init_buffer = []
    INIT_TIMES = 50
    print("Wait Altitude Initialing...")
    for i in range(INIT_TIMES):
        init_buffer.append(bmp280.altitude)
        if (i+1)%10 == 0:print((i+1)*2,"%",sep="")
    init_altitude = sum(init_buffer)/INIT_TIMES
    print("Done OK")
    time.sleep(1)

    ############################# EBIMU 프로세스 시작 ###############################

    eb_p = Process(target=ebimu_process, args=(1,))
    eb_p.start()

    ################################## 실행 로직 ###################################

    while True:

        try:
            (raw_data, parsed_data) = ubr.read()
            print(parsed_data)
            # Write parsed_data to the file
            gf.write(str(parsed_data) + "\n")
        except:
            print("GPS Fail")
            gf.write("GPS Fail \n")

        ################################## 고도 수집 ###################################

        # 로컬 고도 계산
        altitude = bmp280.altitude - init_altitude 
        # 이상치 판단
        if -10 <= altitude <= ESTIMATED_MAX_ALTITUDE :
            is_outlier = False
        else:
            is_outlier = True
        # 데이터 리스트에 추가
        if not is_outlier: 
            datas.append(altitude) 
        else: 
            print("Outlier data")

        mean = np.mean(datas[-WINDOW:])
        moving_averages.append(mean)
        print("Altitude: {:.2f}".format(altitude))
        f.write(f"{datetime.datetime.now()} Altitude: {altitude} m, Is_outlier: {is_outlier}\n")

        ################################## 방향 판단 ###################################

        if moving_averages[-1] > NO_DEPLOY_ALTITUDE:
            is_valid_falling = True

        if  len(moving_averages) > 2 and moving_averages[-2]>moving_averages[-1]: 

            if is_valid_falling:
                falling_count += 1
                status["way"] = "DOWN("+str(falling_count)+")"
            else:
                status["way"] = "DOWN(invalid)"

            print("DOWN", falling_count)
        else: 
            falling_count = 0
            print("UP")
            status["way"] = "UP("+str(is_valid_falling)+")"
            
        ############################# 강제 사출 조건 검사 #################################    

        if ser.in_waiting > 0:
            try:
                read_data = ser.readline().decode()
                print("Received: '"+read_data+"'")
            except:
                read_data = " "
                print("Received a word but Fail to decode")
            time.sleep(1)
            if "<E>" in read_data:
                status["parachute"] = "2"
                pwm.ChangeDutyCycle(9.5)
                time.sleep(2)
                pwm.ChangeDutyCycle(7.5)
                print("Forced Deploy!")
                s.write(f"{datetime.datetime.now()} Servo open: {altitude} m, Type: Force\n")

        ############################# 자동 사출 조건 검사 #################################   

        if not is_deployed and falling_count > FALLING_CONFIRMATION:
            is_deployed = True
            status["parachute"] = "1"
            pwm.ChangeDutyCycle(9.5)
            time.sleep(2)
            pwm.ChangeDutyCycle(7.5)
            print("Deploy!")
            s.write(f"{datetime.datetime.now()} Servo open: {altitude} m, Type: Auto\n")

        ############################### 통신 데이터 가공 ###################################   

        # status["ebimu"] = ", ".join(map(str, eb_data_arr))
        status["bmp"] = str(int(altitude))

        # 추가해야함.

        try:
            status_values = list(status.values())
            total_message = "/".join(map(str, status_values)) + ";"
            for i in range(0, len(total_message), 55):
                message = total_message[i:i + 55]
                ser.write(message.encode())
                print("ok")
        except:
            print("Transmission failure")
        
        if keyboard.is_pressed("space"):
            print("Program exited")
            break
        
############################### 그래프 출력 ###################################

SHOW_GRAPH = False
if SHOW_GRAPH:
    y = datas[WINDOW:]
    x = range(WINDOW//2,len(datas)-WINDOW+WINDOW//2)
    y2 = moving_averages
    x2 = range(len(moving_averages))
    plt.scatter(x2[-1],y2[-1],color="red", marker="*",s=500,label='Deploy!')
    plt.plot(x, y, color="red")
    plt.plot(x2,y2,color="blue")
    plt.axhline(y=NO_DEPLOY_ALTITUDE,color="green")
    plt.xlabel("Data Count")
    plt.ylabel("Altitude")
    plt.title("Parachute_deployment")
    plt.show()
