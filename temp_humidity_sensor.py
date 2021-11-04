
import time
from periphery import I2C

class SHTC3:
    SHT_I2CADDR = 0x70
    SHT_WAKEUP = 0x3517
    SHT_SLEEP = 0xB098
    SHT_NORMAL_TFIRST = 0x7866
    SHT_PART_ID = 0xEFC8

    def __get_word(self, high, low):
        return ((high & 0xFF) << 8) + (low & 0xFF)

    def __temp_calc(self, temp_raw):
        temp = -45+175*temp_raw/(2**16)
        return temp

    def __humidity_calc(self, humidity_raw):
        humidity = 100*humidity_raw/(2**16)
        return humidity

    def __init__(self, i2c_bus='/dev/i2c-0'):
        self.i2c_bus = i2c_bus
        self.i2c = I2C('/dev/i2c-0')
        self.time_to_take_measurment_ms = 12.1

    def get_part_id(self):
        # read part id
        query = [I2C.Message([self.SHT_PART_ID>>8, self.SHT_PART_ID&0xFF]), I2C.Message([0x00]*2, read=True)]
        self.i2c.transfer(self.SHT_I2CADDR, query)
        part_id = self.__get_word(query[1].data[0], query[1].data[1])
        return part_id

    def get_temp_humidity(self):

        #send wake up
        self.i2c.transfer(self.SHT_I2CADDR, [I2C.Message([self.SHT_WAKEUP>>8, self.SHT_WAKEUP&0xFF])])

        #send take measurment
        self.i2c.transfer(self.SHT_I2CADDR, [I2C.Message([self.SHT_NORMAL_TFIRST>>8, self.SHT_NORMAL_TFIRST&0xFF])])

        #wait for measurmenet to complete. Note 1.25x is a buffer
        time.sleep(self.time_to_take_measurment_ms*1.25/1000)

        # read the measurment
        query = [I2C.Message([0x00]*6, read=True)]
        self.i2c.transfer(self.SHT_I2CADDR, query)

        temp_raw = self.__get_word(query[0].data[0], query[0].data[1])
        #temp_crc = query[0].data[2]

        humidity_raw = self.__get_word(query[0].data[3], query[0].data[4])
        #humidity_crc = query[0].data[5]

        # TO DO: check crc

        # convert raw temp humidity reading to actual values
        temp = self.__temp_calc(temp_raw)
        humidity = self.__humidity_calc(humidity_raw)

        #send sleep
        self.i2c.transfer(self.SHT_I2CADDR, [I2C.Message([self.SHT_SLEEP>>8, self.SHT_SLEEP&0xFF])])

        return temp, humidity

def main():
    sensor = SHTC3()
    print(sensor.get_part_id())
    while True:
        print(sensor.get_temp_humidity())
        time.sleep(1)

if __name__ == "__main__":
    main()
