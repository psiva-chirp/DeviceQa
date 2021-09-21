import serial
from time import sleep

ser = serial.Serial ("/dev/ttyS0", 115200)    #Open port with baud rate
fid = open('temp_radar_config.txt', 'r')
for line in fid:
    ser.write((line).encode())
    sleep(0.03)
    read_buffer = ser.read(ser.in_waiting)
    sleep(0.03)
    print(read_buffer)

'''
Manual test:

python3 -m serial.tools.miniterm /dev/ttyS0 115200
'''