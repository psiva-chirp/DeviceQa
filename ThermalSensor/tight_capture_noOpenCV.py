import time
import numpy as np
from htpa_i2c import *
import copy
import json
import Queue as queue
import threading

BUF_SIZE = 250
VIDEO_QUEUE = queue.Queue(BUF_SIZE)
STOP_ALL_THREADS = False

def thermal_data_producer(name, htpa_device):

    while True:
        try:
            #pixel_values, ts, ptats, vdd, elec_offset = htpa_device.get_ondemand_frame()
            pixel_values, ts, ptats, vdd, elec_offset = htpa_device.get_continuous_frame()
            VIDEO_QUEUE.put((pixel_values, ts, ptats, vdd, elec_offset))
            if STOP_ALL_THREADS:
                break
        except KeyboardInterrupt:
            break
    htpa_device.close()

    print('stopped thermal sensor')


with open('TempTables.json','r') as fid:
    temp_table = json.load(fid)

i = 0
dev = HTPA_i2c()
dev_calib = HTPA_calib(temp_table, dev.get_calib_params())

dev_capture_thread = threading.Thread(target=thermal_data_producer, args=(1, dev, ))
dev_capture_thread.daemon = True
dev_capture_thread.start()

last_time = None
running_sum = 0.0
while(True):
    try:
        if not VIDEO_QUEUE.empty():
            pixel_values, ts, ptats, vdd, elec_offset = VIDEO_QUEUE.get()

            im, ambient_temp = dev_calib.calib_image(pixel_values, ptats, vdd, elec_offset)

            if last_time is not None:
                diff_time = ts - last_time
                running_sum += diff_time
                print('Timing, diff & avg')
                print(diff_time)
                print(running_sum / i)
            last_time = ts

            print('Ambient Temperature: %f C' % (ambient_temp/10.0 - 273.15))

            #dK to K
            im /= 10
            #K to C
            im -= 273.15
            print(im[16,16]) # use to get spot temperature measurment of objects
            
            # Normalize image in the temperature range 20C to 40C
            min_valid_temp_C = 20
            max_valid_temp_C = 40
            im -= min_valid_temp_C
            im /= (max_valid_temp_C-min_valid_temp_C)
            im[im<0] = 0
            im[im>1] = 1

            # get colour coded image
            im = im*255
            im = im.astype(np.uint8)
            i += 1
    except KeyboardInterrupt:
        STOP_ALL_THREADS = True
        break


dev_capture_thread.join()

