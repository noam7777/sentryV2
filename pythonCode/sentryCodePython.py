import serial
import time
import numpy as np


# Configure the serial connection
arduino_port = '/dev/ttyUSB0'  # Replace with the correct port
baud_rate = 9600
serial_conn = serial.Serial(arduino_port, baud_rate)
time.sleep(2)  # Wait for the connection to initialize

def send_command(elevation_velocity, azimuth_velocity, should_shoot):
    """
    Sends a structured command to the Arduino.
    :param elevation_velocity: Velocity for the elevation servo.
    :param azimuth_velocity: Velocity for the azimuth servo.
    :param should_shoot: 1 to shoot, 0 otherwise.
    """
    command = f"{elevation_velocity},{azimuth_velocity},{should_shoot}\n"
    serial_conn.write(command.encode())
    print(f"Sent command: {command.strip()}")

try:
    i = 0
    while True:
        # Example: Send a velocity command with no shooting
        i = i + 0.1
        print(i)
        # Example: Vary velocities over time
        elevation_rate = 5*np.sin(i)  # Example: 30°/s
        azimuth_rate =  5*np.sin(i)   # Example: -20°/s
        if(i%5 < 0.1):
            send_command(elevation_rate, azimuth_rate, 1)
        else :
            send_command(elevation_rate, azimuth_rate, 0)

        # Example: Send a command to shoot
        time.sleep(0.1)  # Allow time for the shooting sequence to complete

except KeyboardInterrupt:
    print("Stopping communication.")
finally:
    serial_conn.close()
