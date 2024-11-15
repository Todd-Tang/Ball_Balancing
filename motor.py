import serial
ser = serial.Serial('/dev/serial0', 115200, timeout=1)
ser.write(b'$$')
#print(ser.readline())
print(ser.read(10))