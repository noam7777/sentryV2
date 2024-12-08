#test passed, related arduino code path: Arduino/sentryCodeArduino/controAllActuatorsV2ddd

import serial
import time

# Configure the serial connection
arduino_port = '/dev/ttyUSB0'  # Replace with your Arduino's serial port
baud_rate = 9600
serial_conn = serial.Serial(arduino_port, baud_rate)
time.sleep(2)  # Wait for the connection to initialize

def send_command(elevation_velocity, azimuth_velocity, shoot):
    """
    Sends a velocity command and shoot command to the Arduino.
    :param elevation_velocity: Velocity for the elevation servo (int, degrees/sec).
    :param azimuth_velocity: Velocity for the azimuth servo (int, degrees/sec).
    :param shoot: 1 to shoot, 0 otherwise (int).
    """
    command = f"{elevation_velocity},{azimuth_velocity},{shoot}\n"
    serial_conn.write(command.encode())
    print(f"Sent command: {command.strip()}")

def main():
    try:
        print("Starting motor control test...")

        # Test 1: Move both motors forward without shooting
        print("Test 1: Moving servos forward without shooting...")
        send_command(20, 15, 0)  # Elevation: 20 deg/sec, Azimuth: 15 deg/sec, Shoot: 0
        time.sleep(2)

        # Test 2: Move both motors backward without shooting
        print("Test 2: Moving servos backward without shooting...")
        send_command(-20, -15, 0)  # Elevation: -20 deg/sec, Azimuth: -15 deg/sec, Shoot: 0
        time.sleep(2)

        # Test 3: Stop both motors and shoot
        print("Test 3: Stopping servos and shooting...")
        send_command(0, 0, 1)  # Elevation: 0 deg/sec, Azimuth: 0 deg/sec, Shoot: 1
        time.sleep(2)

        # Test 4: Stop both motors without shooting
        print("Test 4: Stopping servos without shooting...")
        send_command(0, 0, 0)  # Elevation: 0 deg/sec, Azimuth: 0 deg/sec, Shoot: 0
        time.sleep(2)

    except KeyboardInterrupt:
        print("Test interrupted.")
    finally:
        serial_conn.close()
        print("Serial connection closed.")

if __name__ == "__main__":
    main()
