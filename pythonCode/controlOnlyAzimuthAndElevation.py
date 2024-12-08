import serial
import time

# Configure the serial connection
arduino_port = '/dev/ttyUSB0'  # Replace with your Arduino's serial port
baud_rate = 9600
serial_conn = serial.Serial(arduino_port, baud_rate)
time.sleep(2)  # Wait for the connection to initialize

def send_command(elevation_velocity, azimuth_velocity):
    """
    Sends a velocity command to the Arduino.
    :param elevation_velocity: Velocity for the elevation servo (int, degrees/sec).
    :param azimuth_velocity: Velocity for the azimuth servo (int, degrees/sec).
    """
    command = f"{elevation_velocity},{azimuth_velocity}\n"
    serial_conn.write(command.encode())
    print(f"Sent command: {command.strip()}")

def main():
    try:
        print("Starting motor control test...")

        while True:
            # Example 1: Move both motors forward
            print("Moving servos forward...")
            send_command(20, 15)  # Elevation: 20 deg/sec, Azimuth: 15 deg/sec
            time.sleep(2)

            # Example 2: Move both motors backward
            print("Moving servos backward...")
            send_command(-20, -15)  # Elevation: -20 deg/sec, Azimuth: -15 deg/sec
            time.sleep(2)

            # Example 3: Stop both motors
            print("Stopping servos...")
            send_command(0, 0)  # Elevation: 0 deg/sec, Azimuth: 0 deg/sec
            time.sleep(2)

    except KeyboardInterrupt:
        print("Test interrupted.")
    finally:
        serial_conn.close()
        print("Serial connection closed.")

if __name__ == "__main__":
    main()
