import serial
import tkinter as tk
from tkinter import ttk
import threading
import time

class ArduinoReader:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_conn = None
        self.running = True

    def connect(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=1)
            time.sleep(2)  # Allow time for the connection to initialize
        except serial.SerialException as e:
            print(f"Error connecting to Arduino: {e}")

    def read_data(self):
        if self.serial_conn and self.serial_conn.in_waiting > 0:
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                return line
            except Exception as e:
                print(f"Error reading data: {e}")
        return None

    def send_command(self, elevation, azimuth, gun_command):
        if self.serial_conn:
            try:
                command = f"{elevation},{azimuth},{gun_command}\n"
                self.serial_conn.write(command.encode('utf-8'))
            except Exception as e:
                print(f"Error sending command: {e}")

    def close(self):
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()

class ArduinoGUI:
    def __init__(self, root, arduino_reader):
        self.root = root
        self.arduino_reader = arduino_reader

        self.root.title("Arduino Data Display")
        self.root.geometry("400x400")

        # Initialize velocity and shooting state
        self.azimuth_velocity = 0
        self.elevation_velocity = 0
        self.gun_command = 0

        # Labels to display parsed data
        self.gun_state_label = ttk.Label(root, text="Gun State: --", font=("Arial", 14))
        self.gun_state_label.pack(pady=10)

        self.darts_left_label = ttk.Label(root, text="Darts Left: --", font=("Arial", 14))
        self.darts_left_label.pack(pady=10)

        self.azimuth_label = ttk.Label(root, text="Azimuth: --", font=("Arial", 14))
        self.azimuth_label.pack(pady=10)

        self.elevation_label = ttk.Label(root, text="Elevation: --", font=("Arial", 14))
        self.elevation_label.pack(pady=10)

        # Buttons for commands
        self.shutdown_button = ttk.Button(root, text="SHUTDOWN", command=lambda: self.set_gun_command(0))
        self.shutdown_button.pack(pady=5)

        self.disarm_button = ttk.Button(root, text="DISARM", command=lambda: self.set_gun_command(1))
        self.disarm_button.pack(pady=5)

        self.arm_button = ttk.Button(root, text="ARM", command=lambda: self.set_gun_command(2))
        self.arm_button.pack(pady=5)

        self.fire_button = ttk.Button(root, text="FIRE", command=lambda: self.set_gun_command(3))
        self.fire_button.pack(pady=5)

        # Bind keys for azimuth and elevation control
        self.root.bind("<KeyPress>", self.key_press)
        self.root.bind("<KeyRelease>", self.key_release)

        self.start_reading_thread()
        self.start_command_publish_thread()

    def set_gun_command(self, command):
        self.gun_command = command

    def publish_command(self):
        while self.arduino_reader.running:
            self.arduino_reader.send_command(self.elevation_velocity, self.azimuth_velocity, self.gun_command)
            time.sleep(0.2)  # Publish commands every 0.1 seconds

    def key_press(self, event):
        if event.keysym == "w":
            self.elevation_velocity = 10
        elif event.keysym == "s":
            self.elevation_velocity = -10
        elif event.keysym == "a":
            self.azimuth_velocity = -10
        elif event.keysym == "d":
            self.azimuth_velocity = 10
        elif event.keysym == "Return":
            self.set_gun_command(3)  # FIRE command

    def key_release(self, event):
        if event.keysym in ("w", "s"):
            self.elevation_velocity = 0
        elif event.keysym in ("a", "d"):
            self.azimuth_velocity = 0
        elif event.keysym == "Return":
            self.set_gun_command(0)  # Reset FIRE command

    def parse_data(self, data):
        try:
            if data.startswith("S:"):
                parts = data.split(",")
                gun_state = parts[0].split(":")[1]
                darts_left = parts[1].split(":")[1]
                azimuth = parts[2].split(":")[1]
                elevation = parts[3].split(":")[1]
                return gun_state, darts_left, azimuth, elevation
        except Exception as e:
            print(f"Error parsing data: {e}")
        return None, None, None, None

    def update_gui(self, gun_state, darts_left, azimuth, elevation):
        self.gun_state_label.config(text=f"Gun State: {gun_state}")
        self.darts_left_label.config(text=f"Darts Left: {darts_left}")
        self.azimuth_label.config(text=f"Azimuth: {azimuth}")
        self.elevation_label.config(text=f"Elevation: {elevation}")

    def read_from_arduino(self):
        while self.arduino_reader.running:
            data = self.arduino_reader.read_data()
            if data:
                print(data)
                gun_state, darts_left, azimuth, elevation = self.parse_data(data)
                if gun_state is not None:
                    self.update_gui(gun_state, darts_left, azimuth, elevation)
            time.sleep(0.1)  # Small delay to avoid overwhelming the CPU

    def start_reading_thread(self):
        thread = threading.Thread(target=self.read_from_arduino, daemon=True)
        thread.start()

    def start_command_publish_thread(self):
        thread = threading.Thread(target=self.publish_command, daemon=True)
        thread.start()

if __name__ == "__main__":
    # Replace with your Arduino's port
    ARDUINO_PORT = "/dev/ttyUSB0"  # Update as needed
    BAUD_RATE = 9600

    arduino_reader = ArduinoReader(ARDUINO_PORT, BAUD_RATE)
    arduino_reader.connect()

    root = tk.Tk()
    gui = ArduinoGUI(root, arduino_reader)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        arduino_reader.close()
