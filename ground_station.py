import time
import keyboard
import os

IS_TEST = True

if not IS_TEST:
    import serial
    ser = serial.Serial(
        port="/dev/ttyAMA0",
        baudrate=19200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )
HOLD_TIME = 1
buffer = ""
current_time = time.time()
is_first_press = True
press_start_time = 0
if IS_TEST: buffer = "Not deploy/UP/0, 0/0, 0, 0/0/0, 0, 0"
while True:
    if IS_TEST or ser.in_waiting > 0:

        if not IS_TEST:
            try:
                read_data = ser.read().decode()
            except:
                read_data = ""

        if IS_TEST or read_data == ';':
            message = buffer.strip().split("/")
            if len(message) == 6:
                parachute, way, gps, ebimu, bmp, bno = message
                elapsed_time = time.time() - current_time

                os.system('clear')
                print("+-----------------------+")
                print("|  GOAT Rocket Program  |")
                print("+-----------------------+")
                print(f" TIME:\t {elapsed_time:.1f}s")
                print(f" PARA:\t {parachute}")
                print(f" WAY:\t {way}")
                print(f" GPS:\t {gps}")
                print(f" EBIMU:\t {ebimu}")
                print(f" BMP:\t {bmp} m")
                print(f" BNO:\t {bno}")
                print("+-----------------------+")
            else:
                pass
                #print("Invalid data format")
            if not IS_TEST: buffer = ""
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
                if not IS_TEST: ser.write("E".encode())
                is_first_press = True
                time.sleep(1)
    elif keyboard.is_pressed("t"): # 키 입력 테스트
        print("Key input confirmed")
    else:
        is_first_press = True
    time.sleep(0.001)

