import time
import board
import adafruit_bmp280
import RPi.GPIO as GPIO
import numpy as np
import serial
import datetime
import multiprocessing
import sys
import matplotlib.pyplot as plt

# EBIMU 프로세스
def ebimu_process(n):
    # EBIMU 센서 값 저장 파일 이름 생성(당시 시간으로 파일 이름)
    nowTime = str(datetime.datetime.now())
    fileName = nowTime[:10]+"_ebimu_"+nowTime[11:21]
    # 파일 열기
    try:
        f = open(fileName, 'w')
        log = open("log.txt",'w')
    except:
        print("Failed to open file, EBIMU")
        sys.exit() #파일 열기가 실패했다면 강제 종료
        
    # 시리얼 통신 연결
    ser = serial.Serial('/dev/ttyUSB0',115200,timeout=0.001)

    # 버퍼 생성
    buf = "" 
    while True:
        # 센서 값이 들어왔다면 반복
        while ser.inWaiting():
            data = str(ser.read()).strip() # 데이터 입력
            buf += data # 버퍼링(buf변수에 계속 연결)
            if data[3] == "n": # 만약 b'n'데이터가 들어왔다면, 데이터 추출 시작
                # buf에는 "b'0'b'1'b'2'"과 같이 저장 되어 있음, '과 b를 없에줌.
                buf = buf.replace("'","")
                buf = buf.replace("b","") 
            
                # 데이터 파싱
                try : roll, pitch, yaw, x, y, z = map(float,buf[1:-4].split(','))
                except Exception as e:
                    print("Error from data processing : ", e)
                    log.write("Error from data processing : "+str(e)+"\n")
                    buf = ""
                    continue
                
                # 파일에 기록
                datas = [roll,pitch,yaw,x,y,z]
                writeString = "*"+str(datas)[1:-1]+"\n"
                f.write(writeString)
                
                # 출력
                print(roll,pitch,yaw,x,y,z)
                buf = ""

if __name__ == '__main__':
    # EBIMU 프로세스 시작
    eb_p = multiprocessing.Process(target=ebimu_process, args=(1,))
    eb_p.start()

    WINDOW = 10
    THRESHOLD = 20 # 이상치 임계값
    NO_DEPLOY_ALTITUDE = 1
    FALLING_CONFIRMATION = 3

    datas = []  
    moving_averages = []
    falling_count = 0

    #Bmp280 센서 연결
    i2c = board.I2C()  
    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)
    bmp280.sea_level_pressure = 1013.25 # 표준 대기압으로 설정

    #서보 모터 설정
    GPIO.setmode(GPIO.BCM)#핀 모드 설정
    servo_pin = 18
    GPIO.setwarnings(False)
    GPIO.setup(servo_pin, GPIO.OUT)
    pwm = GPIO.PWM(servo_pin, 50)
    pwm.start(0) 

    now = str(datetime.datetime.now())
    fileN = now[:10]+"_bmp_"+now[11:21]
    fileS = now[:10]+"_servoLog_"+now[11:21]
    f = open(fileN, 'w')
    s = open(fileS, 'w')

    ##################################
    # 서보 동작 테스트 코드 추가 공간
    ##################################

    # 센서 초기화
    init_buffer = []
    INIT_TIMES = 50
    print("Wait Altitude Initialing...")
    for i in range(INIT_TIMES):
        init_buffer.append(bmp280.altitude)
        if (i+1)%10 == 0:print((i+1)*2,"%",sep="")
    init_altitude = sum(init_buffer)/INIT_TIMES
    print("Done OK")
    time.sleep(1)

    # 데이터 초기값 임의 지정
    for i in range(WINDOW-1):datas.append(0.1)
    datas.append(0.2)

    #Bmp280
    while True:
        altitude = bmp280.altitude - init_altitude # 로컬 고도 계산
        
        # 이상치 탐지, Z-score기법 사용
        mean = np.mean(datas[-WINDOW:])
        std = np.std(datas[-WINDOW:])
        z = (altitude-mean)/std 
        print('Z-score : {:.2f}'.format(z))
        print("Altitude: {:.2f}".format(altitude))
        if z > THRESHOLD: 
            f.write(f"{datetime.datetime.now()} Outlier Altitude: {altitude} m\n")
            print('Detected outlier with : {:.2f}'.format(altitude))
        else: 
            f.write(f"{datetime.datetime.now()} Altitude: {altitude} m\n")
            datas.append(altitude) # 데이터 리스트에 추가
            moving_averages.append(mean)

        # 상승, 하강 판단
        if  len(moving_averages) > 2 and moving_averages[-2]>moving_averages[-1]: 
            falling_count += 1
            print("DOWN", falling_count)
        else: 
            falling_count = 0
            print("UP")
       
       # 낙하산 사출 조건 검사
        if falling_count > FALLING_CONFIRMATION and moving_averages[-1] > NO_DEPLOY_ALTITUDE:
            pwm.ChangeDutyCycle(9.5)
            time.sleep(2)
            pwm.ChangeDutyCycle(7.5)
            print("Deploy!")
            s.write(f"{datetime.datetime.now()} Servo open: {altitude} m\n")
            break
        

# 그래프 출력
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
