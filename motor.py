import serial
ser = serial.Serial('/dev/serial0', 115200, timeout=1)
print(ser.name)
ser.write(b'$$')
ser.close()
#print(ser.readline())
print(ser.read(10))