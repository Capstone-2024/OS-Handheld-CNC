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
            # self.arduino.flushInput()
            # self.arduino.flushOutput()

        except serial.SerialException as e:
            print(f"Error: {e}")

        self.data = None
        self.output = None

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
            port, baudrate=115200
        )
        arduino.flush()

        # arduino.write('H'.encode('ascii'))
        arduino.write('A'.encode())
        time.sleep(1)
        # data = arduino.readline()
    
        data = arduino.readline()
        print(data)

        arduino.close()

    except serial.SerialException as e:
        print(f"Error: {e}")


    # arduino = ArduinoComms()
    # # arduino.start_transmit()
    # arduino.ardu_write('A'.encode('ascii'))
    # # arduino.start_read()
    # arduino.ardu_read(10)
    # print(arduino.arduino.readline())
    # print(arduino.data)