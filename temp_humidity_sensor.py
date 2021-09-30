
from periphery import I2C
import time

def get_word(high, low):
    return ((high & 0xFF) << 8) + (low & 0xFF)

def temp_calc(temp_raw):
    temp = -45+175*temp_raw/(2**16)
    return temp

def humidity_calc(humidity_raw):
    humidity = 100*humidity_raw/(2**16)
    return humidity

device_address = 0x70
wakeup = 0x3517
sleep = 0xB098
nomral_T_first_clk_stretch_dis = 0x7866
part_id = 0xEFC8
time_to_take_measurment_ms = 12.1

i2c = I2C('/dev/i2c-0')

# read part id
query = [I2C.Message([part_id>>8, part_id&0xFF]), I2C.Message([0x00]*2, read=True)]
i2c.transfer(device_address, query)
print(query[1].data)

#send wake up
i2c.transfer(device_address, [I2C.Message([wakeup>>8, wakeup&0xFF])])

#send take measurment
i2c.transfer(device_address, [I2C.Message([nomral_T_first_clk_stretch_dis>>8, nomral_T_first_clk_stretch_dis&0xFF])])

#wait for measurmenet to complete. Note 1.25x is a buffer
time.sleep(time_to_take_measurment_ms*1.25/1000)

# read the measurment
query = [I2C.Message([0x00]*6, read=True)]
i2c.transfer(device_address, query)

temp_raw = get_word(query[0].data[0], query[0].data[1])
temp_crc = query[0].data[2]

humidity_raw = get_word(query[0].data[3], query[0].data[4])
humidity_crc = query[0].data[5]

temp = temp_calc(temp_raw)
humidity = humidity_calc(humidity_raw)

print(temp)
print(humidity)