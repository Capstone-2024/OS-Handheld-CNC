import serial.tools.list_ports
import serial
from threading import Thread
import struct 
import sys
import time

"""
ERROR and STATUS CODE

STATUS CODES
-----------------------
Link Status
L0 - standby
L1 - Arduino request for data
L2 - 
L3 - 

Z Status
Z0 - standby
Z1 - Arduino request for data
Z2 - 

Safety Button Status 
B0 - 
B1 -


Accelerometer Status
A0 - standby
A2 - 

-----------------------
ERROR CODES

"""

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
            self.arduino.flushInput()
            self.arduino.flushOutput()

        except serial.SerialException as e:
            print(f"Error: {e}")

        self.data = None
        self.output = None
        # # Sequential Status Check
        # if self.data[1] == 0:
        #     self.status = 1 # Do nothing
        #     print(self.status)
        # # elif self.data[1]==1:
        # #     self.ardu_write(self.data + outbound_data)
        # elif self.data[1] == 3:
        #     self.status = 0

    def start_transmit(self):
        Thread(target=self.ardu_write, args=())
        return self

    def ardu_write(self, output):
        self.output = output
        self.arduino.write(output)

    def start_read(self):
        Thread(target=self.ardu_read, args=())
        return self

    def ardu_read(self, size):
        self.data = self.arduino.read(size).decode()

    # def ardu_read(self, size): 
    #     data = self.arduino.read(size)
    #     print(data)
    #     self.data = struct.unpack('ff', data)
        
    def close(self):
        self.arduino.close()


if __name__ == "__main__":
    port = ''

    for device in serial.tools.list_ports.comports(): 
        print(device.description)
        if 'USB Serial' in device.description: 
            port = device.device
            
    try:
        arduino = serial.Serial(
            port, 115200, timeout=1
        )
        arduino.flushInput()
        arduino.flushOutput()

        arduino.write('A'.encode('ascii'))
        time.sleep(1)
        if arduino.in_waiting:
            # data = arduino.readline().decode()
            data = arduino.read(8)
            data1, data2 = struct.unpack('ff', data)
            
            print(data)
        else: 
            print("No Data")

    except serial.SerialException as e:
        print(f"Error: {e}")


    # arduino = ArduinoComms()
    # arduino.start_transmit()
    # arduino.ardu_write('A'.encode('ascii'))
    # arduino.ardu_read(4)
    # print(arduino.data)

    # arduino.ardu_read(4)
    # print(arduino.data)