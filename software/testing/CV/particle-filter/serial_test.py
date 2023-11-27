import serial
import serial.tools.list_ports
import threading
import time
import random

class ArduinoCommunication:
    def __init__(self, baudrate=11520, timeout=1):
        port = ''
        for device in serial.tools.list_ports.comports(): 
            if 'CH340' in device.description: 
                port = device.device

        self.output = None

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None
        self.serial_lock = threading.Lock()
        self.communication_thread = threading.Thread(target=self._communication_thread)

    def connect(self):
        try:
            self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Error: {e}")

    def disconnect(self):
        if self.serial_connection:
            self.serial_connection.close()
            print(f"Disconnected from {self.port}.")

    def write_data(self, data):
        with self.serial_lock:
            if self.serial_connection:
                self.serial_connection.write(data.encode())
                print(f"Sent: {data}")

    def read_data(self, num_bytes=1):
        with self.serial_lock:
            if self.serial_connection:
                data = self.serial_connection.read(num_bytes)
                print(f"Received: {data.decode()}")
                return data.decode()

    def start_communication_thread(self):
        if not self.communication_thread.is_alive():
            self.communication_thread.start()

    def _communication_thread(self):
        # Write data to Arduino every 2 seconds
        while True:
            self.write_data(self.output)

if __name__ == "__main__":
    arduino = ArduinoCommunication()

    # Connect to Arduino
    arduino.connect()

    # Start communication thread
    arduino.start_communication_thread() 

    try:
        # Your main calculations or other tasks here
        while True:
            print("Main thread doing calculations...")
            time.sleep(1)
    except KeyboardInterrupt:
        # Disconnect from Arduino and wait for the communication thread to finish
        arduino.disconnect()
        arduino.communication_thread.join()
        print("Exiting program.")