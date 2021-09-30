import sys

sys.path.append('./ThermalSensor')

from htpa_i2c import *
from light_sensor import LTR329ALS01
from temp_humidity_sensor import SHTC3

thermal_sensor = HTPA_i2c()
light_sensor = LTR329ALS01()
temp_sensor = SHTC3()

slow_sensor_interval_ms = 1000

prev_thermal_ts = None
last_light_temp_ts = None
running_sum = 0.0
num_measurement = 0
while True:
    pixel_values, ts, ptats, vdd, elec_offset = thermal_sensor.get_ondemand_frame()

    time_since_light_temp = slow_sensor_interval_ms
    if last_light_temp_ts is not None:
        time_since_light_temp = last_light_temp_ts - ts
    
    if time_since_light_temp >= slow_sensor_interval_ms:
        light = light_sensor.lux()
        temp, humidity = temp_sensor.get_temp_humidity()
        print('Light: %.2f, Temp: %.2f, Humidity: %.2f' % (light, temp, humidity))
        last_light_temp_ts = ts
    
    if prev_thermal_ts is not None:
        data_interval = ts - prev_thermal_ts
        running_sum += data_interval
        num_measurement += 1

        print('Diff: %.1f, %.1f' % (data_interval, running_sum/num_measurement))

    prev_thermal_ts = ts
