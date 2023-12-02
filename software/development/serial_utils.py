import time
from pySerialTransfer import pySerialTransfer as txfer
import serial.tools.list_ports
from sys import platform
import random

class ArduinoComms:
    """
    Initialzes the Arduino Serial communication. Uses the pySerialTransfer library.
    """

    def __init__(self, baudrate=115200):
        self.link = self.start_arduino_comms(baudrate)

    def start_arduino_comms(self, baudrate):
        port = ""
        for device in serial.tools.list_ports.comports():
            print(platform)
            if platform == "linux":
                port = '/dev/ttyUSB0'
            else:
                port = 'COM7'
        link = txfer.SerialTransfer(port, baud=baudrate)
        link.open()
        time.sleep(2)  # allow some time for the Arduino to completely reset
        return link

    def prompt_accel(self):
        # try:
        # Keep track of packet size
        send_size = 0
        # Send 'A' to start transfer
        str_ = "A"
        str_size = self.link.tx_obj(str_, send_size) - send_size
        send_size += str_size
        self.link.send(send_size)

    def get_accel(self): 
        """ Wait for a response and report any errors while receiving packets """
        while not self.link.available():
            if self.link.status < 0:
                if self.link.status == txfer.CRC_ERROR:
                    print("ERROR: CRC_ERROR")
                elif self.link.status == txfer.PAYLOAD_ERROR:
                    print("ERROR: PAYLOAD_ERROR")
                elif self.link.status == txfer.STOP_BYTE_ERROR:
                    print("ERROR: STOP_BYTE_ERROR")
                else:
                    print("ERROR: {}".format(self.link.status))
        # Parse response list
        rec_float_ = self.link.rx_obj(obj_type=float, obj_byte_size=4)
        rec_float_2_ = self.link.rx_obj(obj_type=float, obj_byte_size=4, start_pos=(4))
        print("SENT: {}".format(str_))
        print("RCVD: {} {}".format(rec_float_, rec_float_2_))
        # print(" ")
        return rec_float_, rec_float_2_

    def send_error(self, x, y):
        """
        Sends the appropriate error vector for each time cycle.
        """
        # Keep track of packet size
        send_size = 0
        # Send Error Vector
        str_ = "I"
        str_size = self.link.tx_obj(str_, send_size) - send_size
        send_size += str_size
        x_size = self.link.tx_obj(x, send_size) - send_size
        send_size += x_size
        y_size = self.link.tx_obj(y, send_size) - send_size
        send_size += y_size
        self.link.send(send_size)

    def home(self):
        # Keep track of packet size
        send_size = 0
        # Send 'H' to home
        str_ = "H"
        str_size = self.link.tx_obj(str_, send_size) - send_size
        send_size += str_size
        self.link.send(send_size)
        
        """ Wait for a response and report any errors while receiving packets """
        while not self.link.available():
            if self.link.status < 0:
                if self.link.status == txfer.CRC_ERROR:
                    print("ERROR: CRC_ERROR")
                elif self.link.status == txfer.PAYLOAD_ERROR:
                    print("ERROR: PAYLOAD_ERROR")
                elif self.link.status == txfer.STOP_BYTE_ERROR:
                    print("ERROR: STOP_BYTE_ERROR")
                else:
                    print("ERROR: {}".format(self.link.status))

        # Parse response list
        incoming_str_ = self.link.rx_obj(obj_type=str, obj_byte_size=1)
        print("SENT: {}".format(str_))
        print("RCVD: {}".format(incoming_str_))
        return incoming_str_


if __name__ == "__main__":
    arduino_communicator = ArduinoComms()

    while True:
        if arduino_communicator.home() == 'G': 
            # arduino_communicator.read_accel()
            # arduino_communicator.prompt_accel()
            # arduino_communicator.get_accel()
            # x = round(random.uniform(0, 3), 2)
            # y = round(random.uniform(0, 3), 2)
            x = 5
            y = 5
            arduino_communicator.send_error(x, y)
            time.sleep(0.2)
            # arduino_communicator.send_error(0, 0)
            # time.sleep(1)
            # arduino_communicator.send_error(x, y)
            print(x, y)