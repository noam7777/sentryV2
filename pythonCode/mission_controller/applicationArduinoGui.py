import serial
import tkinter as tk
from tkinter import ttk
import threading
import time
from trackFaceAndControlArduino import LockAndShootController
import cv2

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
        else :
            print("no serial conn")
    def close(self):
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()

from PIL import Image, ImageTk

class ArduinoGUI:
    def __init__(self, root, arduino_reader):
        self.root = root
        self.arduino_reader = arduino_reader

        self.root.title("Arduino Data Display")
        self.root.geometry("800x600")  # Adjusted for video display

        # Initialize velocity and shooting state
        self.manual_azimuth_velocity = 0
        self.manual_elevation_velocity = 0
        self.gun_command = 0
        self.should_send_commands_manually = True
        self.lastSentCommandTimeStamp = 0

        # Initialize LockAndShootController
        self.lock_and_shoot_controller = LockAndShootController(arduino_reader.serial_conn)
        self.lock_and_shoot_controller.update_gui_callback = self.update_video_frame

        # Start auto control thread
        self.lock_and_shoot_controller.start_auto_control_thread()

        # Labels to display parsed data
        self.gun_state_label = ttk.Label(root, text="Gun State: --", font=("Arial", 14))
        self.gun_state_label.pack(pady=10)

        self.darts_left_label = ttk.Label(root, text="Darts Left: --", font=("Arial", 14))
        self.darts_left_label.pack(pady=10)

        self.azimuth_label = ttk.Label(root, text="Azimuth: --", font=("Arial", 14))
        self.azimuth_label.pack(pady=10)

        self.elevation_label = ttk.Label(root, text="Elevation: --", font=("Arial", 14))
        self.elevation_label.pack(pady=10)

        self.mode_label = ttk.Label(root, text="mode: AUTO", font=("Arial", 14))
        self.mode_label.pack(pady=10)

        # Buttons for commands
        self.shutdown_button = ttk.Button(root, text="SHUTDOWN", command=lambda: self.set_gun_command(0))
        self.shutdown_button.pack(pady=5)

        self.disarm_button = ttk.Button(root, text="DISARM", command=lambda: self.set_gun_command(1))
        self.disarm_button.pack(pady=5)

        self.arm_button = ttk.Button(root, text="ARM", command=lambda: self.set_gun_command(2))
        self.arm_button.pack(pady=5)

        self.fire_button = ttk.Button(root, text="FIRE", command=lambda: self.set_gun_command(3))
        self.fire_button.pack(pady=5)

        self.fire_button = ttk.Button(root, text="RELOAD", command=lambda: self.set_gun_command(4))
        self.fire_button.pack(pady=5)

        self.lock_and_shoot_button = ttk.Button(root, text="AUTO_MODE", command=lambda: self.setAutoMode(True))
        self.lock_and_shoot_button.pack(pady=5)

        # Create a toggle button
        self.precised_shot_toggle_button = tk.Button(root, text="OFF", bg="red", command=self.toggle_precised_shot_button, width=10, height=2)
        self.precised_shot_toggle_button.pack(pady=20)
        self.precised_shot_toggle_button.is_on = False  # Start with OFF

        self.manual_button = ttk.Button(root, text="MANUAL_MODE", command=lambda: self.setAutoMode(False))
        self.manual_button.pack(pady=5)

        # Video display Canvas
        self.canvas = tk.Canvas(root, width=640, height=480, bg="black")
        self.canvas.pack(pady=10)

        # Bind keys for azimuth and elevation control
        self.root.bind("<KeyPress>", self.key_press)
        self.root.bind("<KeyRelease>", self.key_release)

        self.start_reading_thread()
        self.start_command_sending_thread()

    def toggle_precised_shot_button(self):
        # Toggle the button state and text
        if self.precised_shot_toggle_button.is_on:
            self.precised_shot_toggle_button.is_on = False
            self.lock_and_shoot_controller.shouldPerformPrecisedShoot = False
            self.precised_shot_toggle_button.config(text="OFF", bg="red")
        else:
            self.precised_shot_toggle_button.is_on = True
            self.lock_and_shoot_controller.shouldPerformPrecisedShoot = True
            self.precised_shot_toggle_button.config(text="ON", bg="green")
    
    def setAutoMode(self, shouldBeInAutoMode) :
        self.should_send_commands_manually = False
        self.lock_and_shoot_controller.shouldSendCommandsToRobot = False
        if (shouldBeInAutoMode):
            self.lock_and_shoot_controller.shouldSendCommandsToRobot = True
        else:
            self.should_send_commands_manually = True




    def set_gun_command(self, command):
        self.gun_command = command
        self.send_commands()

    def send_commands(self):
        self.lastSentCommandTimeStamp = time.time()

        if self.should_send_commands_manually :
            self.arduino_reader.send_command(self.manual_elevation_velocity, self.manual_azimuth_velocity, self.gun_command)
        else:
            auto_elevation_velocity, auto_azimuth_velocity, auto_gun_command =  self.lock_and_shoot_controller.sentryController.currentCommand
            self.arduino_reader.send_command(auto_elevation_velocity, auto_azimuth_velocity, auto_gun_command)

    def key_press(self, event):
        if event.keysym == "w":
            self.manual_elevation_velocity = -30  # Adjust speed as needed
        elif event.keysym == "s":
            self.manual_elevation_velocity = 30
        elif event.keysym == "a":
            self.manual_azimuth_velocity = 30
        elif event.keysym == "d":
            self.manual_azimuth_velocity = -30
        elif event.keysym == "Return":
            self.set_gun_command(3)  # FIRE command

    def key_release(self, event):
        if event.keysym in ("w", "s"):
            self.manual_elevation_velocity = 0
        elif event.keysym in ("a", "d"):
            self.manual_azimuth_velocity = 0
        elif event.keysym == "Return":
            self.gun_command = 2  # Reset FIRE command

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
        self.mode_label.config(text=f"auto mode: {not(self.should_send_commands_manually)}")

    def update_video_frame(self, frame):
        # Convert frame (OpenCV BGR) to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb_frame)
        imgtk = ImageTk.PhotoImage(image=img)

        # Update canvas
        self.canvas.create_image(0, 0, anchor=tk.NW, image=imgtk)
        self.canvas.imgtk = imgtk  # Store reference to avoid garbage collection

    def read_from_arduino(self):
        while self.arduino_reader.running:
            data = self.arduino_reader.read_data()
            if data:
                gun_state, darts_left, azimuth, elevation = self.parse_data(data)
                if gun_state is not None:
                    self.update_gui(gun_state, darts_left, azimuth, elevation)
            time.sleep(0.1)  # Small delay to avoid overwhelming the CPU

    def send_commands_task(self):
        while self.arduino_reader.running:
            current_time = time.time()
            if (current_time -  self.lastSentCommandTimeStamp) > 0.1:
                self.send_commands()  # Send a message to keep the Arduino active
                time.sleep(0.2)  # Send every 1 second
                

    def start_reading_thread(self):
        thread = threading.Thread(target=self.read_from_arduino, daemon=True)
        thread.start()

    def start_command_sending_thread(self):
        thread = threading.Thread(target=self.send_commands_task, daemon=True)
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
        gui.lock_and_shoot_controller.cap.release()

