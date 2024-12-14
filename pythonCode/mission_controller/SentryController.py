import time
import serial

class Controller:
    def __init__(self, serial_conn, baud_rate=9600, kP_azimuth=80, kP_elevation=120):
        self.kP_azimuth = kP_azimuth
        self.kP_elevation = kP_elevation
        self.targetErrorToShoot = 0.1
        self.serial_conn = serial_conn
        time.sleep(2)  # Allow connection to initialize
        self.currentCommand = (0, 0, 2)

    def sentry_pid(self, avg_position, is_face_detected, isCameraCenterInBbox):
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

        gunCommand = 3 if (isCameraCenterInBbox) else 2  # 3 for shoot and 2 for arm

        cmd_azimuth = int(errorX * self.kP_azimuth)
        cmd_elevation = int(errorY * self.kP_elevation)
        self.currentCommand = (cmd_elevation, cmd_azimuth, gunCommand)
    
    def close(self):
        self.serial_conn.close()