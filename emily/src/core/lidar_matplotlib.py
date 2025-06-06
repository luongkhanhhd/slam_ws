from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import csv
import time
import serial.tools.list_ports

def find_lidar_port():
    """Automatically detect the LIDAR USB port."""
    ports = serial.tools.list_ports.comports()
    print("Available ports:", [port.device for port in ports])
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
            lidar = RPLidar(port_name)
            print("LIDAR Info:", lidar.get_info())
            print("Health Status:", lidar.get_health())
            print("Starting LIDAR motor and warming up for 90 seconds...")
            lidar.start_motor()
            time.sleep(90)
            return lidar
        except Exception as e:
            print(f"Connection error (retries left: {retries}): {str(e)}")
            retries -= 1
            time.sleep(2)
            if retries == 0:
                raise Exception("Failed to connect to LIDAR after retries")

def scan_and_plot():
    """Scan LIDAR data and plot in real-time."""
    port = find_lidar_port()
    if not port:
        print("No LIDAR port detected! Exiting...")
        return
    print(f"Using port: {port}")
    lidar = None
    
    # Initialize plot
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='polar')
    ax.set_title("LIDAR Scan")
    
    # Initialize CSV
    with open('lidar_data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Scan', 'Quality', 'Angle', 'Distance'])
        
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
                        print(f"Angle: {angle:.2f} do, Distance: {distance:.2f}mm, Quality: {quality}")
                
                # Update plot
                ax.clear()
                ax.scatter(np.radians(angles), distances, s=5)
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
            plt.ioff()
            plt.show()

if __name__ == "__main__":
    scan_and_plot()
