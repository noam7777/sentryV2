import time
import serial

class Controller:
    def __init__(self, port='/dev/ttyUSB0', baud_rate=9600, kP_azimuth=80, kP_elevation=120):
        self.kP_azimuth = kP_azimuth
        self.kP_elevation = kP_elevation
        self.targetErrorToShoot = 0.1
        self.targetCenteredCountToShoot = 5
        self.serial_conn = serial.Serial(port, baud_rate)
        time.sleep(2)  # Allow connection to initialize
        self.targetCenteredCounter = 0

    def send_command(self, elevation_velocity, azimuth_velocity, should_shoot):
        """
        Sends a structured command to the Arduino.
        """
        command = f"{elevation_velocity},{azimuth_velocity},{should_shoot}\n"
        self.serial_conn.write(command.encode())
        print(f"Sent command: {command.strip()}")

    def sentry_pid(self, avg_position, face_detected):
        """
        PID control logic for directing the robot.
        """
        targetPosX, targetPosY = avg_position
        errorX = -(targetPosX - 0.5)
        errorY = (targetPosY - 0.5)
        
        if (((errorX ** 2 + errorY ** 2) ** 0.5) < self.targetErrorToShoot):
            self.targetCenteredCounter += 1
        else:
            self.targetCenteredCounter = 0

        should_shoot = 1 if (self.targetCenteredCounter > self.targetCenteredCountToShoot and face_detected) else 0

        cmd_azimuth = int(errorX * self.kP_azimuth)
        cmd_elevation = int(errorY * self.kP_elevation)
        self.send_command(cmd_elevation, cmd_azimuth, should_shoot)
    
    def read_data(self):
        if self.serial_conn.in_waiting > 0:
            try:
                raw_data = self.serial_conn.readline()
                print(f"Raw data: {raw_data}")  # Debugging line to print raw bytes
                data = raw_data.decode('utf-8', errors='ignore').strip()
                return data
            except Exception as e:
                print(f"Error reading data: {e}")
        return None
    
    def close(self):
        self.serial_conn.close()