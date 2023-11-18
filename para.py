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

if __name__ == '__main__':
    WINDOW = 10
    THRESHOLD= 20 # 이상치 임계값
    NO_DEPLOY_ALTITUDE= 1
    FALLING_CONFIRMATION = 3

    datas = []  
    moving_averages = []
    falling_count = 0

    #Bmp280 센서 연결
    i2c = board.I2C()  
    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)
    bmp280.sea_level_pressure = 1013.25 # 표준 대기압으로 설정

    # 센서 초기화
    init_buffer = []
    init_times = 50
    print("Wait Altitude Initialing...")
    for i in range(init_times):init_buffer.append(bmp280.altitude)
    init_altitude = sum(init_buffer)/init_times
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
        if z > THRESHOLD: 
            print('#####Detected outlier with : {:.2f}#####'.format(altitude))
        else: 
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
            print("Deploy!")
            break

        # 고도 출력
        print("Altitude: {:.2f}".format(altitude))

# 그래프 출력
y = datas[20:]
x = range(WINDOW//2,len(datas)-20+WINDOW//2)
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
