import serial
import time
#ser = serial.Serial('/dev/serial0', 115200, timeout=1)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
print(ser.name)
ser.write(b'$$')
time.sleep(1)
#print(ser.readline())


message = ser.readline()
while ("ok" not in message and "error" not in message):
    print(message)
    message = ser.readline()