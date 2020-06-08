import os
import time
from datetime import date
import board
import digitalio

def disk_usage():
    path = '/'
    st = os.statvfs(path)
    bytes_avail = (st.f_bavail * st.f_frsize)
    megabytes = bytes_avail / 1024 / 1024
    return megabytes

# set up motion sensor
pir_sensor = digitalio.DigitalInOut(board.D18)
pir_sensor.direction = digitalio.Direction.INPUT

prev_signal = None
while True:
    if disk_usage() <= 1:
        print('Not enough disk space')
        break

    current_signal = pir_sensor.value

    if prev_signal is None or current_signal != prev_signal:
        filename = '/home/pi/PIR_' + date.today().strftime("%d_%m_%Y") + '.txt'
        ts = int(time.time()*1000)
        current_signal_digit = 0
        if current_signal:
            current_signal_digit = 1
        with open(filename, "a+") as fid:
            fid.write('%d,%d\n' % (ts, current_signal_digit))
        print('%d,%d' % (ts, current_signal_digit))
        prev_signal = pir_sensor.value

    time.sleep(0.5)
