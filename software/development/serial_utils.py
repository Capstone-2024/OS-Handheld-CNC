import serial.tools.list_ports
import serial
from threading import Thread
import struct 
import sys
import time

# class ArduinoComms:
#     def __init__(self, baudrate=115200, timeout=1):

#         port = ''
#         for device in serial.tools.list_ports.comports(): 
#             print(device.description)
#             if 'USB Serial' in device.description: 
#                 port = device.device
                

#         self.arduino = None
#         try:
#             self.arduino = serial.Serial(
#                 port, baudrate, timeout=timeout
#             )
#             print(f"Connected to {port} at {baudrate} baud.")

#         except serial.SerialException as e:
#             print(f"Error: {e}")

#         self.data = None
#         self.lineData = None
#         self.output = None

#     def start_transmit(self):
#         Thread(target=self.ardu_write, args=())
#         return self

#     def ardu_write(self, output):
#         self.output = output
#         self.arduino.write(output)

#     def thread_read(self):
#         Thread(target=self.ardu_read, args=()).start()
#         return self
    
#     def thread_readline(self): 
#         Thread(target=self.ardu_readline, args=()).start()
#         return self

#     def ardu_read(self, size):
#         self.data = self.arduino.read(size).decode()

#     def ardu_readline(self): 
#         self.lineData = self.arduino.readline()
        
#     def close(self):
#         self.arduino.close()

import serial.tools.list_ports
import serial
from threading import Thread
import struct
import sys
import time

class ArduinoComms:
    def __init__(self, baudrate=115200, timeout=1):
        port = ''
        for device in serial.tools.list_ports.comports():
            print(device.description)
            if 'USB Serial' in device.description:
                port = device.device

        self.arduino = None
        try:
            self.arduino = serial.Serial(
                port, baudrate, timeout=timeout
            )
            print(f"Connected to {port} at {baudrate} baud.")

        except serial.SerialException as e:
            print(f"Error: {e}")

        self.data = None
        self.lineData = None
        self.output = None

    def start_transmit(self):
        Thread(target=self.ardu_write, args=(self.output,)).start()
        return self

    def ardu_write(self, output):
        self.output = output
        self.arduino.write(output)

    def start_read(self, size):
        Thread(target=self.ardu_read, args=(size,)).start()
        return self

    def start_readline(self):
        Thread(target=self.ardu_readline).start()
        return self

    def ardu_read(self, size):
        self.data = self.arduino.read(size)

    def ardu_readline(self):
        self.lineData = self.arduino.readline().decode()

    def close(self):
        self.arduino.close()


if __name__ == "__main__":
    arduino = ArduinoComms()
    arduino.start_transmit()
    time.sleep(1)
    arduino.ardu_write('A'.encode('ascii'))  # Send 'A' to Arduino
    time.sleep(2)  # Wait for Arduino to send data
    arduino.start_read(8)  # Read 8 bytes of binary data
    time.sleep(1)  # Wait for the read thread to finish
    z_accel, y_accel = struct.unpack('ff', arduino.data)
    print(f"Z Acceleration: {z_accel}, Y Acceleration: {y_accel}")


# if __name__ == "__main__":
    # port = ''

    # for device in serial.tools.list_ports.comports(): 
    #     print(device.description)
    #     if 'USB Serial' in device.description: 
    #         port = device.device
            
    # arduino = serial.Serial(port, baudrate=115200)
    # arduino.write('H'.encode('ascii'))
    # time.sleep(0.5)
    # arduino.write('S'.encode('ascii'))


    # arduino = ArduinoComms()
    # arduino.start_transmit()
    # # arduino.ardu_write('A'.encode('ascii'))
    # arduino.ardu_write('H'.encode('ascii'))
    # time.sleep(1)
    # arduino.ardu_write('A'.encode('ascii'))
    # time.sleep(1)
    # arduino.thread_readline()
    # # arduino.ardu_read(8)
    # print(arduino.ardu_readline())
    # # print(arduino.data)