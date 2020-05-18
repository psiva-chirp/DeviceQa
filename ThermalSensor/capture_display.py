import time
import numpy as np
import cv2
from htpa import *
import copy
import json

with open('TempTables.json','r') as fid:
    temp_table = json.load(fid)

i = 0

dev = HTPA()
last_time = None
running_sum = 0.0
while(True):
    print("Capturing image " + str(i))
    #if (i == 5):
    #    dev.measure_observed_offset()

    im = dev.get_image()
    current_time = time.time()
    if last_time is not None:
        diff_time = current_time - last_time
        running_sum += diff_time
        print('Timing, diff & avg')
        print(diff_time)
        print(running_sum / i)
    last_time = current_time
    #print(pixel_values)
    print('Pix range')
    print(np.min(im))
    print(np.max(im))
    '''
    im = pixel_values
    
    im_temp_comp = copy.deepcopy(im)
    im_temp_comp = cv2.resize(im_temp_comp, None, fx=12, fy=12)    
    im_temp_comp -= np.min(im_temp_comp)
    im_temp_comp /= np.max(im_temp_comp)
    im_temp_comp = im_temp_comp*255
    im_temp_comp = im_temp_comp.astype(np.uint8)
    im_temp_comp = cv2.applyColorMap(im_temp_comp, cv2.COLORMAP_JET)
    cv2.imshow('temp', im_temp_comp)
    im = dev.elec_offset_compensation(im)
    #im = dev.temperature_compensation(im, ptats)
    #im = dev.sensitivity_compensation(im)
    '''
    # resize and scale image to make it more viewable on raspberry pi screen
    im = cv2.resize(im, None, fx=12, fy=12)    
    print(np.min(im))
    print(np.max(im))
    im -= -35
    im /= (150+35)
    im[im<0] = 0
    im[im>1] = 15
    #im -= np.min(im)
    #im /= np.max(im)
    im = im*255
    im = im.astype(np.uint8)
    im = cv2.applyColorMap(im, cv2.COLORMAP_JET)
    cv2.imshow('frame', im)
    i += 1

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

dev.close()

cv2.destroyAllWindows()
