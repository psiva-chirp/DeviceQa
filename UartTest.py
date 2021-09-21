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
ser.write(('sensorStop\n').encode())
sleep(0.03)
read_buffer = ser.read(ser.in_waiting)
sleep(0.03)
print(read_buffer)
'''

'''
while True:
    received_data = ser.read()              #read serial port
    sleep(0.03)
    data_left = ser.inWaiting()             #check for remaining byte
    received_data += ser.read(data_left)
    print (received_data)                   #print received data
'''