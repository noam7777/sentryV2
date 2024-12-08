import serial
import time

# Configure the serial connection
arduino_port = '/dev/ttyUSB0'  # Replace with the correct port for your Arduino
baud_rate = 9600
serial_conn = serial.Serial(arduino_port, baud_rate)
time.sleep(2)  # Allow the connection to initialize

def send_command(elevation_velocity, azimuth_velocity, should_shoot):
    """
    Sends a structured command to the Arduino.
    :param elevation_velocity: Elevation angular velocity in DecidegreesPerSec (integer).
    :param azimuth_velocity: Azimuth angular velocity in DecidegreesPerSec (integer).
    :param should_shoot: 1 to shoot, 0 otherwise (integer).
    """
    command = f"{elevation_velocity},{azimuth_velocity},{should_shoot}\n"
    serial_conn.write(command.encode())
    print(f"Sent command: {command.strip()}")

try:
    print("Testing Arduino Commands...")
    
    # Test 1: Elevation and Azimuth velocities with no shooting
    print("Test 1: Moving eleveation without shooting...")
    send_command(100, 0, 0)  # Move servos at 10°/sec and -10°/sec
    time.sleep(2)
    
    # Test 2: Elevation and Azimuth velocities with shooting
    print("Test 2: Moving azimoth servos and initiating shoot sequence...")
    send_command(0, 150, 1)  # Move servos at 15°/sec and initiate shooting
    time.sleep(5)  # Wait for the shoot sequence to complete
    
    # Test 3: Stop shooting and return to idle
    print("Test 3: Stop shooting...")
    send_command(0, 0, 0)  # Stop all motions and shooting
    time.sleep(2)
    
    # Test 4: Continuous shooting
    print("Test 4: Continuous shooting...")
    for _ in range(3):
        send_command(0, 0, 1)  # Move servos and keep shooting
        time.sleep(3)
    send_command(0, 0, 0)  # Stop after tests
    time.sleep(2)
    
    print("All tests completed.")

except KeyboardInterrupt:
    print("Testing interrupted.")
finally:
    serial_conn.close()
    print("Serial connection closed.")
