import time
import serial

class Controller:
    def __init__(self, serial_conn, baud_rate=9600, kP_azimuth=80, kP_elevation=120):
        self.kP_azimuth = kP_azimuth
        self.kP_elevation = kP_elevation
        self.targetErrorToShoot = 0.1
        self.targetCenteredCountToShoot = 5
        self.serial_conn = serial_conn
        time.sleep(2)  # Allow connection to initialize
        self.targetCenteredCounter = 0
        self.currentCommand = (0, 0, 2)

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
        # Maintain the last command time
        if not hasattr(self, 'last_command_time'):
            self.last_command_time = 0

        current_time = time.time() * 1000  # Convert to milliseconds

        self.last_command_time = current_time

        targetPosX, targetPosY = avg_position
        errorX = -(targetPosX - 0.5)
        errorY = (targetPosY - 0.5)

        if (((errorX ** 2 + errorY ** 2) ** 0.5) < self.targetErrorToShoot):
            self.targetCenteredCounter += 1
        else:
            self.targetCenteredCounter = 0

        gunCommand = 3 if (self.targetCenteredCounter > self.targetCenteredCountToShoot and face_detected) else 2  # 3 for shoot and 2 for arm

        cmd_azimuth = int(errorX * self.kP_azimuth)
        cmd_elevation = int(errorY * self.kP_elevation)
        self.currentCommand = (cmd_elevation, cmd_azimuth, gunCommand)
        # self.send_command(cmd_elevation, cmd_azimuth, should_shoot)


    # not the roll of the controller to read data from the arduino
    # def read_data(self):
    #     if self.serial_conn.in_waiting > 0:
    #         try:
    #             raw_data = self.serial_conn.readline()
    #             print(f"Raw data: {raw_data}")  # Debugging line to print raw bytes
    #             data = raw_data.decode('utf-8', errors='ignore').strip()
    #             return data
    #         except Exception as e:
    #             print(f"Error reading data: {e}")
    #     return None
    
    def close(self):
        self.serial_conn.close()