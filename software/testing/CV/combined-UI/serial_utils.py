import serial.tools.list_ports
import serial

'''
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

'''


def control_status(read_data, send_data): 
    status = 1

    if read_data[1] == 0: 
        status = 1 # Do nothing 
    elif read_data[1]==1: 
        ardu_write(read_data + send_data)
    elif read_data[1] == 3: 
        status = 0 

    return status

def retract_Z(): 
    ardu_write("Z0")

def ardu_write(data):
    # Find Arduino Device Port
    port = None

    # Ensure that Arduino is read from /wrote to
    for device in serial.tools.list_ports.comports(): 
        if 'Arduino' in device.description: 
            port = device.device

    arduino = serial.Serial(port)
    arduino.write(data)
    arduino.close()

def ardu_read():
    port = None

    # Ensure that Arduino is read from /wrote to
    for device in serial.tools.list_ports.comports(): 
        if 'Arduino' in device.description: 
            port = device.device

    arduino = serial.Serial(port)
    data = arduino.read()
    arduino.close()

    # Read Arduino Outputs
    return data