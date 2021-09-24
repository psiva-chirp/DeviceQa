import serial
from time import sleep

ser = serial.Serial ("/dev/ttyS3", 115200)    #Open port with baud rate
fid = open('temp_radar_config.txt', 'r')
for line in fid:
    n = 11
    chunks = [line[i:i+n] for i in range(0, len(line), n)]
    for c in chunks:
        ser.write(c.encode('utf-8'))
        sleep(0.03)
    read_buffer = ser.read(ser.in_waiting)
    sleep(0.03)
    print(read_buffer)

'''
Manual test:

python3 -m serial.tools.miniterm /dev/ttyS0 115200
'''