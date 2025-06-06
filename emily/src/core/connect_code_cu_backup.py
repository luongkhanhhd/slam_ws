from rplidar import RPLidar
import time
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import numpy as np
import csv
import os

# Serial connection to STM32
#MCU_PORT = '/dev/ttyUSB2'
MCU_PORT = '/dev/ttyAMA10'
MCU_BAUDRATE = 9600

def check_port_permissions(port_name):
    """Check and attempt to fix port permissions."""
    try:
        if not os.access(port_name, os.R_OK | os.W_OK):
            print(f"No read/write access to {port_name}, attempting to fix...")
            os.system(f"sudo chmod 666 {port_name}")
            if not os.access(port_name, os.R_OK | os.W_OK):
                raise Exception(f"Failed to gain access to {port_name}")
            print(f"Permissions fixed for {port_name}")
    except Exception as e:
        print(f"Permission error: {str(e)}")
        raise

def init_mcu_serial():
    """Initialize serial connection to STM32."""
    check_port_permissions(MCU_PORT)
    try:
        mcu_serial = serial.Serial(MCU_PORT, MCU_BAUDRATE, timeout=1)
        print(f"Connected to STM32 on {MCU_PORT}")
        return mcu_serial
    except serial.SerialException as e:
        print(f"Failed to connect to STM32: {e}")
        exit(1)

def find_lidar_port():
    """Automatically detect the LIDAR USB port."""
    ports = serial.tools.list_ports.comports()
    print("Available ports:", [port.device for port in ports])
    for port in ports:
        if port.device == '/dev/ttyUSB0':
            print(f"Selected LIDAR port: {port.device}")
            return port.device
    for port in ports:
        if "USB" in port.device or "ACM" in port.device:
            print(f"Selected LIDAR port: {port.device}")
            return port.device
    print("No suitable USB port found for LIDAR")
    return None

def safe_lidar_operation(port_name):
    """Safely initialize LIDAR connection with retry mechanism."""
    retries = 3
    while retries > 0:
        try:
            check_port_permissions(port_name)
            lidar = RPLidar(port_name)
            print("LIDAR Info:", lidar.get_info())
            print("Health Status:", lidar.get_health())
            print("Starting LIDAR motor and warming up for 30 seconds...")
            lidar.start_motor()
            time.sleep(20)
            return lidar
        except Exception as e:
            print(f"Connection error (retries left: {retries}): {str(e)}")
            retries -= 1
            time.sleep(2)
            if retries == 0:
                raise Exception("Failed to connect to LIDAR after retries")

def calculate_path(angles, distances, target_x, target_y, current_x, current_y, current_theta):
    """Calculate velocity and angular velocity using VFH."""
    SAFE_DISTANCE = 500  # mm
    MAX_SPEED = 100  # mm/s
    MAX_OMEGA = 90   # degrees/s
    
    regions = [float('inf')] * 12
    for angle, distance in zip(angles, distances):
        if distance > 0:
            region = int(angle // 30) % 12
            regions[region] = min(regions[region], distance)
    
    dx = target_x - current_x
    dy = target_y - current_y
    target_angle = np.arctan2(dy, dx) * 180 / np.pi
    
    target_region = int(target_angle // 30) % 12
    if regions[target_region] > SAFE_DISTANCE:
        desired_angle = target_angle
    else:
        free_regions = [i for i, d in enumerate(regions) if d > SAFE_DISTANCE]
        if not free_regions:
            print("No free path found, stopping...")
            return 0, 0, 0
        desired_angle = min(free_regions, key=lambda x: abs(x * 30 - target_angle)) * 30
    
    angle_diff = (desired_angle - current_theta) % 360
    if angle_diff > 180:
        angle_diff -= 360
    omega = max(min(angle_diff * 2, MAX_OMEGA), -MAX_OMEGA)
    if abs(angle_diff) < 15:
        vx = MAX_SPEED * np.cos(np.radians(desired_angle))
        vy = MAX_SPEED * np.sin(np.radians(desired_angle))
    else:
        vx, vy = 0, 0
    
    return vx, vy, omega

def scan_lidar_data():
    """Scan LIDAR, plan path, and control robot."""
    mcu_serial = init_mcu_serial()
    port = find_lidar_port()
    if not port:
        print("No LIDAR port detected! Exiting...")
        return
    print(f"Using port: {port}")
    lidar = None
    
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='polar')
    ax.set_title("LIDAR Scan")
    
    with open('lidar_data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Scan', 'Quality', 'Angle', 'Distance'])
        
        try:
            x_start = float(input("Enter starting x (mm): "))
            y_start = float(input("Enter starting y (mm): "))
        except ValueError:
            print("Invalid input, using default (0, 0)")
            x_start, y_start = 0, 0
        
        current_x, current_y, current_theta = x_start, y_start, 0
        target_x, target_y = 2000, 1000
        print(f"Starting at ({current_x}, {current_y}), moving to ({target_x}, {target_y})")
        
        try:
            lidar = safe_lidar_operation(port)
            print("Starting LIDAR scan with default frequency (~5.5Hz)")
            
            for scan_count, scan in enumerate(lidar.iter_scans(max_buf_meas=500)):
                if scan_count % 10 == 0:
                    print(f"Scan #{scan_count}: Received {len(scan)} data points")
                
                angles = []
                distances = []
                for quality, angle, distance in scan:
                    if distance > 0:
                        angles.append(angle)
                        distances.append(distance)
                        writer.writerow([scan_count, quality, angle, distance])
                        print(f"Angle: {angle:.2f}°, Distance: {distance:.2f}mm, Quality: {quality}")
                
                vx, vy, omega = calculate_path(angles, distances, target_x, target_y, current_x, current_y, current_theta)
                
                command = f"{vx:.2f},{vy:.2f},{omega:.2f}\n"
                mcu_serial.write(command.encode())
                print(f"Sent to STM32: {command.strip()}")
                
                dt = 0.5
                current_x += vx * dt
                current_y += vy * dt
                current_theta += omega * dt
                print(f"Position: ({current_x:.2f}, {current_y:.2f}), Theta: {current_theta:.2f}°")
                
                distance_to_target = np.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
                if distance_to_target < 100:
                    print("Reached target!")
                    mcu_serial.write("0,0,0\n".encode())
                    break
                
                ax.clear()
                ax.scatter(np.radians(angles), distances, s=5)
                ax.plot(np.radians([current_theta]), [100], 'ro')
                ax.plot(np.radians([np.arctan2(target_y - current_y, target_x - current_x) * 180 / np.pi]), [100], 'go')
                ax.set_title(f"LIDAR Scan #{scan_count}")
                plt.pause(0.01)
                
        except KeyboardInterrupt:
            print("Scan stopped by user request")
        except Exception as e:
            print(f"Critical error: {str(e)}")
        finally:
            if lidar:
                try:
                    lidar.stop()
                    lidar.stop_motor()
                    lidar.disconnect()
                    print("LIDAR disconnected safely")
                except Exception as e:
                    print(f"Error during disconnection: {str(e)}")
            mcu_serial.write("0,0,0\n".encode())
            mcu_serial.close()
            plt.ioff()
            plt.show()

if __name__ == "__main__":
    scan_lidar_data()
