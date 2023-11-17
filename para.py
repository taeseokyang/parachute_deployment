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

# 프로그램 시작 지점
if __name__ == '__main__':
    #Bmp280 센서 연결
    i2c = board.I2C()  
    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)
    bmp280.sea_level_pressure = 1013.25 # 표준 대기압으로 설정

    window = 10
    init_altitude = 0  # 기준 영점
    threshold = 20 # 이상치 임계값
    no_deploy_altitude = 1
    moving_averages = []

    falling_count = 0
    # 센서 초기화
    init_buffer = []
    init_times = 50
    for i in range(init_times):
        print("Wait Initialing...")
        init_buffer.append(bmp280.altitude)
    init_altitude = sum(init_buffer)/init_times
    datas = []
    for i in range(window-1):
        print("Wait Initialing...")
        datas.append(0.1)
    datas.append(0.2)
    #Bmp280
    while True:
        altitude = bmp280.altitude - init_altitude # 로컬 고도 계산
        # 이상치 탐지, Z-score기법 사용
        mean = np.mean(datas[-window:])
        std = np.std(datas[-window:])
        z = (altitude-mean)/std 
        print('Z-score : {:.2f}'.format(z))
        if z > threshold: 
            print('###########################################Detected outlier with : {:.2f}'.format(altitude))
        else: 
            datas.append(altitude) # 데이터 리스트에 추가
            if len(datas)>window:
                moving_averages.append(mean)

        # 낙하산 사출 조건 검사
        if  len(moving_averages) > 2 and moving_averages[-2]>moving_averages[-1]: 
            falling_count += 1
            print("DOWN", falling_count)
        else: 
            falling_count = 0
            print("UP", falling_count)
       
        falling_confirmation = 3
        if falling_count> falling_confirmation and moving_averages[-1] > no_deploy_altitude:
            try:
                print("Deploy!")
                time.sleep(1)
                break
            except KeyboardInterrupt:
                GPIO.cleanup()

        # 로컬 고도 출력
        print("Altitude: {:.2f}".format(altitude))

y = datas[20:]
x = range(window//2,len(datas)-20+window//2)
y2 = moving_averages
x2 = range(len(moving_averages))
plt.scatter(x2[-1],y2[-1],color="red", marker="*",s=500,label='Deploy!')
plt.plot(x, y, color="red")
plt.plot(x2,y2,color="blue")
plt.axhline(y=no_deploy_altitude,color="green")
plt.xlabel("Data Count")
plt.ylabel("Altitude")
plt.title("Parachute_deployment")
plt.show()
input()
