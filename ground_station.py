import time
import serial
import keyboard

ser = serial.Serial(
    port="/dev/ttyAMA0",
    baudrate=19200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

buffer = ""
current_time = time.time()
is_first_press = True
press_start_time = 0
HOLD_TIME = 2
while True:
    read_data = ser.read().decode()
    # 데이터 마지막 판단
    if read_data == ';':
        # 파싱
        massage = buffer.strip().split("/")
        # 유효성 검사
        if len(massage) == 6:
            parachute, way, gps, ebimu, bmp, bno = massage
            elapsed_time = time.time() - current_time
            print(f"TIME:\t {elapsed_time:.1f}s")
            print(f"PARA:\t {parachute}")
            print(f"WAY:\t {way}")
            print(f"GPS:\t {gps}")
            print(f"EBIMU:\t {ebimu}")
            print(f"BMP:\t {bmp}m")
            print(f"BNO:\t {bno}")
        else:print("Invalid data format")
        # buffer = ""
        print(f"------------------------")

    # 강제 고양이 사출
    if keyboard.is_pressed("space"):
        if is_first_press:
            press_start_time = time.time()
            is_first_press = False
        else:
            time_pressed = time.time() - press_start_time
            print(f"!!Keep press space for Ejection")
            print(f"!!Left time:\t {abs(HOLD_TIME-time_pressed):.1f}s")
            if time_pressed >= HOLD_TIME:
                print("Sended Ejection Message")
                is_first_press = True
                ser.write("E".encode())
    elif keyboard.is_pressed("t"): # 키 입력 테스트
        print("Key input confirmed")
    else:
        is_first_press = True
    time.sleep(0.001)

