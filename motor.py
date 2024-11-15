import serial
#ser = serial.Serial('/dev/serial0', 115200, timeout=1)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
print(ser.name)
ser.write(b'$$')
#print(ser.readline())
print(ser.read(10))