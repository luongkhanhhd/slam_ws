from rplidar import RPLidar
import time
import serial
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
            # Start motor and warm up for 90 seconds (SLAMTEC recommendation)
            print("Starting LIDAR motor and warming up for 20 seconds...")
            lidar.start_motor()
            time.sleep(20)
            return lidar
        except serial.SerialException as e:
            print(f"Connection error (retries left: {retries}): {str(e)}")
            retries -= 1
            time.sleep(2)
            if retries == 0:
                raise Exception("Failed to connect to LIDAR after retries")

def scan_lidar_data():
    """Scan and process LIDAR data."""
    port = find_lidar_port()
    if not port:
        print("No LIDAR port detected! Exiting...")
        return
    print(f"Using port: {port}")
    lidar = None
    
    try:
        lidar = safe_lidar_operation(port)
        
        # Use default scan frequency (~5.5Hz)
        print("Starting LIDAR scan with default frequency (~5.5Hz)")
        
        for scan_count, scan in enumerate(lidar.iter_scans(max_buf_meas=500)):
            if scan_count % 10 == 0:  # Print every 10 scans to reduce load
                print(f"Scan #{scan_count}: Received {len(scan)} data points")
            
            # Process scan data
            for quality, angle, distance in scan:
                if distance > 0:  # Ignore zero-distance readings
                    print(f"Angle: {angle:.2f}°, Distance: {distance:.2f}mm, Quality: {quality}")
                    
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

if __name__ == "__main__":
    scan_lidar_data()
