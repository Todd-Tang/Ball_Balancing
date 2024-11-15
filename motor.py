import serial
import time
#ser = serial.Serial('/dev/serial0', 115200, timeout=1)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
print(ser.name)
time.sleep(1)
print(ser.readall())
ser.write(b'G91\n')

ser.write(b'G0 X0.1 F1\n')
print(ser.readall())

ser.write(b'G0 X0.2 F1\n')
print(ser.readall())

ser.write(b'G0 X-0.1 F1\n')
print(ser.readall())


#time.sleep(1)
#ser.write(b'$$\r\n')
#time.sleep(1)
print(ser.readall())

#print(ser.readline())


"""""
message = ser.readline()
while (b"ok" not in message and b"error" not in message):
    print(message)
    message = ser.readline()
"""""