import serial
import serial.tools.list_ports
from serial.threaded import ReaderThread
import threading
import time
import random
import sys

class PrintLines(LineReader):
    def connection_made(self, transport):
        super(PrintLines, self).connection_made(transport)
        sys.stdout.write('port opened\n')
        self.write_line('hello world')

    def handle_line(self, data):
        sys.stdout.write('line received: {}\n'.format(repr(data)))

    def connection_lost(self, exc):
        if exc:
            traceback.print_exc(exc)
        sys.stdout.write('port closed\n')

if __name__ == "__main__":
    port = ''

    for device in serial.tools.list_ports.comports(): 
        print(device.description)
        if 'USB Serial' in device.description: 
            port = device.device

    ser = serial.serial_for_url('loop://', baudrate=115200, timeout=1)
    with ReaderThread(ser, PrintLines) as protocol:
        protocol.write_line('hello')
        time.sleep(2)