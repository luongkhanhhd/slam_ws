from rplidar import RPLidar
import time
import serial
import serial.tools.list_ports
import pygame
import math
import numpy as np
import csv
import os

# Serial configuration
MCU_BAUDRATE = 9600

def check_port_permissions(port_name):
    """Check and ensure port permissions."""
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

def find_lidar_port():
    """Automatically detect the LIDAR port."""
    ports = serial.tools.list_ports.comports()
    print("Available ports:", [(port.device, port.vid, port.pid, port.description) for port in ports])
    
    for port in ports:
        try:
            check_port_permissions(port.device)
            lidar = RPLidar(port.device)
            info = lidar.get_info()
            lidar.stop()
            lidar.disconnect()
            print(f"Selected LIDAR port: {port.device} (VID:PID={port.vid}:{port.pid}, Desc={port.description})")
            return port.device
        except Exception as e:
            print(f"Port {port.device} not LIDAR: {str(e)}")
    
    print("No suitable USB port found for LIDAR")
    return None

def find_mcu_port(exclude_port=None):
    """Automatically detect the STM32 port, excluding LIDAR port."""
    ports = serial.tools.list_ports.comports()
    stm32_vids_pids = [(0x0403, 0x6001), (0x1A86, 0x7523), (0x0483, 0x5740)]
    
    for port in ports:
        if port.device == exclude_port:
            continue
        try:
            check_port_permissions(port.device)
            test_serial = serial.Serial(port.device, MCU_BAUDRATE, timeout=1)
            test_serial.write("0,0,0\n".encode())
            time.sleep(0.1)
            test_serial.close()
            print(f"Selected STM32 port: {port.device} (VID:PID={port.vid}:{port.pid}, Desc={port.description})")
            return port.device
        except (serial.SerialException, Exception) as e:
            print(f"Port {port.device} not STM32: {str(e)}")
    
    for port in ports:
        if port.device == exclude_port:
            continue
        if (port.vid, port.pid) in stm32_vids_pids:
            print(f"Selected STM32 port by VID:PID: {port.device} (VID:PID={port.vid}:{port.pid}, Desc={port.description})")
            return port.device
    
    print("No suitable port found for STM32")
    return None

def init_mcu_serial(port):
    """Initialize serial connection to STM32."""
    check_port_permissions(port)
    try:
        mcu_serial = serial.Serial(port, MCU_BAUDRATE, timeout=1)
        print(f"Connected to STM32 on {port}")
        return mcu_serial
    except serial.SerialException as e:
        print(f"Failed to connect to STM32: {e}")
        exit(1)

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
            time.sleep(30)
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

def draw_lidar_map(screen, angles, distances, current_x, current_y, current_theta, target_x, target_y):
    """Draw 2D map with Pygame."""
    WIDTH, HEIGHT = 800, 600
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    SCALE = 0.1  # 1mm = 0.1 pixel
    
    screen.fill(WHITE)
    
    for angle, distance in zip(angles, distances):
        if distance > 0:
            rad = math.radians(angle)
            x = current_x + distance * math.cos(rad)
            y = current_y + distance * math.sin(rad)
            pygame.draw.circle(screen, BLACK, (int(x * SCALE + WIDTH/2), int(y * SCALE + HEIGHT/2)), 2)
    
    robot_pos = (int(current_x * SCALE + WIDTH/2), int(y * SCALE + HEIGHT/2))
    pygame.draw.circle(screen, RED, robot_pos, 10)
    theta_rad = math.radians(current_theta)
    end_x = current_x + 50 * math.cos(theta_rad)
    end_y = current_y + 50 * math.sin(theta_rad)
    pygame.draw.line(screen, RED, robot_pos, (int(end_x * SCALE + WIDTH/2), int(end_y * SCALE + HEIGHT/2)), 3)
    
    target_pos = (int(target_x * SCALE + WIDTH/2), int(target_y * SCALE + HEIGHT/2))
    pygame.draw.circle(screen, GREEN, target_pos, 10)
    
    pygame.display.flip()
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False
    return True

def scan_lidar_data():
    """Scan LIDAR, plan path, and control robot."""
    lidar_port = find_lidar_port()
    mcu_port = find_mcu_port(exclude_port=lidar_port)
    
    if not mcu_port or not lidar_port:
        print("Cannot proceed without both MCU and LIDAR ports!")
        return
    
    print(f"Final configuration: STM32 on {mcu_port}, LIDAR on {lidar_port}")
    
    mcu_serial = init_mcu_serial(mcu_port)
    port = lidar_port
    lidar = None
    
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("LiDAR 2D Map")
    
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
                try:
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
                    
                    if not draw_lidar_map(screen, angles, distances, current_x, current_y, current_theta, target_x, target_y):
                        break
                    
                    time.sleep(0.1)
                    
                except serial.SerialException as se:
                    print(f"Serial error during scan: {se}")
                    continue
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
            pygame.quit()

if __name__ == "__main__":
    scan_lidar_data()
