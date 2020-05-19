import time
import numpy as np
import cv2
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
            pixel_values, ts, ptats, vdd, elec_offset = htpa_device.get_ondemand_frame()
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
    if not VIDEO_QUEUE.empty():
        pixel_values, ts, ptats, vdd, elec_offset = VIDEO_QUEUE.get()

        im = dev_calib.calib_image(pixel_values, ptats, vdd, elec_offset)

        if last_time is not None:
            diff_time = ts - last_time
            running_sum += diff_time
            print('Timing, diff & avg')
            print(diff_time)
            print(running_sum / i)
        last_time = ts

        im /= 10
        im -= 273.15
        print(im[16,16])
        im -= 20
        im /= (40-20)
        im[im<0] = 0
        im[im>1] = 1
        #im -= np.min(im)
        #im /= np.max(im)
        im = im*255
        im = im.astype(np.uint8)
        im = cv2.applyColorMap(im, cv2.COLORMAP_JET)
        cv2.imshow('frame', im)
        i += 1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            STOP_ALL_THREADS = True
            break

dev_capture_thread.join()

cv2.destroyAllWindows()
