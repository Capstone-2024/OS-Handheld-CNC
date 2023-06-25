import serial.tools.list_ports
import serial

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