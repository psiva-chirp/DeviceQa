from periphery import I2C
import time

class LTR329ALS01:
    ALS_I2CADDR = 0x29

    ALS_CONTR = 0x80
    ALS_MEAS_RATE = 0x85

    ALS_PART_ID = 0x86
    ALS_MANUFAC_ID = 0x87

    ALS_DATA_CH1_0 = 0x88
    ALS_DATA_CH1_1 = 0x89
    ALS_DATA_CH0_0 = 0x8A
    ALS_DATA_CH0_1 = 0x8B

    ALS_STATUS = 0x8C

    ALS_GAIN_1X = 0x00          #1 to 64k lux (default)
    ALS_GAIN_2X = 0x01          #0.5 to 32k lux
    ALS_GAIN_4X = 0x02          #0.25 to 16k lux
    ALS_GAIN_8X = 0x03          #0.125 to 8k lux
    ALS_GAIN_48X = 0x06         #0.02 to 1.3k lux  [home lighting can range up to ~1k lux]
    ALS_GAIN_96X = 0x07         #0.01 to 600 lux
    ALS_GAIN_VALUES = {
        ALS_GAIN_1X: 1,
        ALS_GAIN_2X: 2,
        ALS_GAIN_4X: 4,
        ALS_GAIN_8X: 8,
        ALS_GAIN_48X: 48,
        ALS_GAIN_96X: 96
    }

    ALS_INT_50ms = 0x01
    ALS_INT_100ms = 0x00
    ALS_INT_150ms = 0x04
    ALS_INT_200ms = 0x02
    ALS_INT_250ms = 0x05
    ALS_INT_300ms = 0x06
    ALS_INT_350ms = 0x07
    ALS_INT_400ms = 0x03
    ALS_INT_VALUES = {
        ALS_INT_50ms: 0.5,
        ALS_INT_100ms: 1,
        ALS_INT_150ms: 1.5,
        ALS_INT_200ms: 2,
        ALS_INT_250ms: 2.5,
        ALS_INT_300ms: 3,
        ALS_INT_350ms: 3.5,
        ALS_INT_400ms: 4
    }

    ALS_RATE_50ms = 0x00
    ALS_RATE_100ms = 0x01
    ALS_RATE_200ms = 0x02
    ALS_RATE_500ms = 0x03
    ALS_RATE_1000ms = 0x04
    ALS_RATE_2000ms = 0x05

    ALS_MAX_WAKEUP_TIME = 0.01

    def __init__(self, i2c_bus='/dev/i2c-0', gain = ALS_GAIN_48X, integration = ALS_INT_100ms, rate = ALS_RATE_500ms):
        self.i2c_bus = i2c_bus
        self.i2c = I2C(i2c_bus)

        self.gain = gain
        self.integration = integration
        self.rate = rate

        time.sleep(0.01)

    def __get_active_cmd(self):
        return ((self.gain & 0x07) << 2) + 0x01

    def __get_standby_cmd(self):
        return ((self.gain & 0x07) << 2) + 0x00

    def __get_meas_rate_cmd(self):
        return ((self.integration & 0x07) << 3) + (self.rate & 0x07)

    def __get_word(self, high, low):
        return ((high & 0xFF) << 8) + (low & 0xFF)

    def __write(self, register, value):
        cmd = [I2C.Message([register, value])]
        self.i2c.transfer(self.ALS_I2CADDR, cmd)

    def __read(self, register):
        query = [I2C.Message([register]), I2C.Message([0x00], read=True)]
        self.i2c.transfer(self.ALS_I2CADDR, query)
        return query[1].data[0]

    def light(self):
        cntrl = self.__read(self.ALS_CONTR)
        als_mode = cntrl % 2
        if als_mode == 0:
            print('sensor in stand-by mode. Sending wake up cmd')
            cmd = self.__get_active_cmd()
            self.__write(self.ALS_CONTR, cmd)
            cmd = self.__get_meas_rate_cmd()
            self.__write(self.ALS_MEAS_RATE, cmd)
            time.sleep(self.ALS_MAX_WAKEUP_TIME)
        else:
            print('sensor already active')

        c1_0 = self.__read(self.ALS_DATA_CH1_0)
        c1_1 = self.__read(self.ALS_DATA_CH1_1)
        c0_0 = self.__read(self.ALS_DATA_CH0_0)
        c0_1 = self.__read(self.ALS_DATA_CH0_1)

        data1 = int(self.__get_word(c1_1, c1_0))
        data0 = int(self.__get_word(c0_1, c0_0))

        return (data0, data1)

    def lux(self):
        # Calculate Lux value from formular in Appendix A of the datasheet
        light_level = self.light()
        if light_level[0]+light_level[1] > 0:
            ratio = light_level[1]/(light_level[0]+light_level[1])
            if ratio < 0.45:
                return (1.7743 * light_level[0] + 1.1059 * light_level[1]) / self.ALS_GAIN_VALUES[self.gain] / self.ALS_INT_VALUES[self.integration]
            elif ratio < 0.64 and ratio >= 0.45:
                return (4.2785 * light_level[0] - 1.9548 * light_level[1]) / self.ALS_GAIN_VALUES[self.gain] / self.ALS_INT_VALUES[self.integration]
            elif ratio < 0.85 and ratio >= 0.64:
                return (0.5926 * light_level[0] + 0.1185 * light_level[1]) / self.ALS_GAIN_VALUES[self.gain] / self.ALS_INT_VALUES[self.integration]
            else:
                return 0
        else:
            return 0

def main():
    sensor = LTR329ALS01()
    while True:
        print(sensor.lux())
        time.sleep(1)

if __name__ == "__main__":
    main()


