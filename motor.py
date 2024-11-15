import serial
import time
#ser = serial.Serial('/dev/serial0', 115200, timeout=1)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
print(ser.name)
time.sleep(1)
ser.write(b'?\n')
#time.sleep(1)
#ser.write(b'$$\r\n')
#time.sleep(1)
response = ser.readlines()
for line in response:
    print(line.strip().decode('utf-8'))

#print(ser.readline())


"""""
message = ser.readline()
while (b"ok" not in message and b"error" not in message):
    print(message)
    message = ser.readline()
"""""