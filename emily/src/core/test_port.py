import serial
import time
import sys

PORT = '/dev/ttyUSB1'
BAUDRATE = 9600
TIMEOUT = 2  # Tăng timeout

def main():
    try:
        print(f"Đang kết nối tới {PORT}...")
        ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
        time.sleep(1)  # Đợi kết nối ổn định
        
        print("Gửi lệnh kiểm tra kết nối...")
        ser.write(b"50,0,0#")
        
        print("Đang đợi phản hồi...")
        start_time = time.time()
        while time.time() - start_time < TIMEOUT:
            if ser.in_waiting:
                response = ser.readline().decode().strip()
                print(f"--> Nhận được: '{response}'")
                break
            time.sleep(0.1)
        else:
            print("!!! Không nhận được phản hồi sau 2 giây !!!")
            print("Kiểm tra lại kết nối phần cứng và baudrate")
        
        ser.close()
    except Exception as e:
        print(f"Lỗi: {str(e)}", file=sys.stderr)

if __name__ == "__main__":
    main()
