import time
import keyboard
import os
import serial
ser = serial.Serial(
    port="/dev/ttyAMA0",
    baudrate=19200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

HOLD_TIME = 0.5
buffer = ""
current_time = time.time()
is_first_press = True
press_start_time = 0

while True:
    if ser.in_waiting > 0:

        try:
            read_data = ser.read().decode()
        except:
            read_data = ""

        if read_data == ';':
            message = buffer.strip().split("/")
            if len(message) == 4:
                parachute, way, ebimu, bmp = message
                elapsed_time = time.time() - current_time

                #os.system('clear')
                print("+-----------------------+")
                print("|  GOAT Rocket Program  |")
                print("+-----------------------+")
                print(f" TIME:\t {elapsed_time:.1f}s")
                print(f" PARA:\t {parachute}")
                print(f" WAY:\t {way}")
                print(f" EBIMU:\t {ebimu}")
                print(f" BMP:\t {bmp} m")
                print("+-----------------------+")
            else:
                pass
                #print("Invalid data format")
            buffer = ""
        else:
            buffer += read_data
            
    # 강제 고양이 사출
    if keyboard.is_pressed("space"):
        if is_first_press:
            press_start_time = time.time()
            is_first_press = False
        else:
            time_pressed = time.time() - press_start_time
            print(f"Keep press space for Ejection")
            print(f"Left time: {abs(HOLD_TIME-time_pressed):.1f}s")
            if time_pressed >= HOLD_TIME:
                print("Sended Ejection Message")
                for i in range(3):
                    ser.write(("<E>"*17+"\r\n").encode())
                is_first_press = True
                time.sleep(0.1)
    elif keyboard.is_pressed("t"): # 키 입력 테스트
        print("Key input confirmed")
    else:
        is_first_press = True
    time.sleep(0.001)
