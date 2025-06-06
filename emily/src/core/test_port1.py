import serial
import time

def send_command(port='/dev/ttyUSB1', baudrate=9600, command="TEST"):
    try:
        
        with serial.Serial(port, baudrate, timeout=1) as ser:
            time.sleep(0.1)
            
            
            full_cmd = f"{command}#"
            ser.write(full_cmd.encode())
            print(f"Sent: {full_cmd}")
            
            
            start = time.time()
            while time.time() - start < 0.5:
                if ser.in_waiting:
                    response = ser.readline().decode().strip()
                    print(f"Received: {response}")
                    return True
            
            print("No response")
            return False
            
    except Exception as e:
        print(f"Loi: {e}")
        return False

if __name__ == "__main__":
    send_command(command="Hello")
