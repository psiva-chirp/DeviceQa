from periphery import I2C
import time
import numpy as np
import copy
import struct
import time
from scipy import interpolate

class HTPA_calib:
    def __init__(self, temp_table, calib_param):
        self.calib_param = calib_param

        self.temp_table = None
        if temp_table is not None:
            for table in temp_table:
                if table['TABLENUMBER'] == self.calib_param['TN']:
                    self.temp_table = table
        if self.temp_table is not None:
            self.temp_interp_f = interpolate.interp2d(self.temp_table['XTATemps'], self.temp_table['YADValues'], self.temp_table['TempTable'], kind='linear')
        else:
            self.temp_interp_f = None
            print('Could not find Table Number %d in temp_table provided' % self.TN)

    def __get_ambient_temperature_dK(self):
        Ta = np.mean(self.ptats) * self.calib_param['PTATgradient'] + self.calib_param['PTAToffset']
        return Ta

    def __temperature_compensation(self, im):
        comp = ((self.calib_param['ThGrad']*np.mean(self.ptats)) / pow(2, self.calib_param['gradScale'])) + self.calib_param['ThOffset']
        Vcomp = im - comp
        return Vcomp
    
    def __vdd_compensation(self, im):
        if self.vdd is None:
            return im
        vdd_av = np.mean(self.vdd)
        ptat_av = np.mean(self.ptats)

        numerator1 = ((self.calib_param['VddCompGrad']*ptat_av)/pow(2, self.calib_param['VddScGrad']) + self.calib_param['VddCompOff'])
        numerator2 = vdd_av - self.calib_param['VDDTh1'] - ((self.calib_param['VDDTh2']-self.calib_param['VDDTh1'])/(self.calib_param['PTATTh2']-self.calib_param['PTATTh1']))*(ptat_av - self.calib_param['PTATTh1'])
        comp = numerator1*numerator2/(pow(2, self.calib_param['VddScOff']))
        im = im - comp
        return im

    def __elec_offset_compensation(self, im):
        return im - self.elec_offset

    def __sensitivity_compensation(self, im):
        PCSCALEVAL = 100000000
        im = (im* PCSCALEVAL)/self.calib_param['PixC']
        return im

    def calib_image(self, pixel_values, ptats, vdd, elec_offset):
        self.ptats = ptats
        self.vdd = vdd
        self.elec_offset = elec_offset
        im = self.__elec_offset_compensation(pixel_values)
        im = self.__temperature_compensation(im)
        im = self.__vdd_compensation(im)
        im = self.__sensitivity_compensation(im)
        Ta = self.__get_ambient_temperature_dK()
        if self.temp_table is not None:
            left_insert_idx = np.searchsorted(self.temp_table['XTATemps'], Ta)
            col_start_end = [left_insert_idx-1, left_insert_idx]
            if col_start_end[1] >= len(self.temp_table['XTATemps']):
                col_start_end[1] = -1
            if col_start_end[0] == -1 or col_start_end[1] == -1:
                # can't do conversion
                return im
            im = im.reshape(-1,)
            out = [self.temp_interp_f(Ta,YY + self.temp_table['TABLEOFFSET']) for YY in im]
            im = np.asarray(out).reshape(32,32)
        return im


class HTPA_i2c:
    def __init__(self, address=0x1A, i2c_bus="/dev/i2c-5"):
        self.address = address
        self.i2c = I2C(i2c_bus)
        self.blockshift = 4

        # data read periodically while imaging
        self.data_ptats = None 
        self.data_vdd = None
        self.elec_offset = None

        # buffer to gather pixel data
        self.ts = None
        self.pixel_values = np.zeros(1024)
        self.header_values = np.zeros(8)

        self.frame_cnt = 0

        print ("Grabbing EEPROM data")
        eeprom = self.__get_eeprom()
        self.calib_params = self.__extract_eeprom_parameters(eeprom)
        self.eeprom = eeprom

        print("Initializing capture settings with stored calibration data")
        wakeup_and_blind = self.__generate_command(0x01, 0x01) # wake up the device
        adc_res = self.__generate_command(0x03, self.calib_params['MBIT_calib']) # set ADC resolution to 16 bits
        pull_ups = self.__generate_command(0x09, self.calib_params['PU_calib'])

        self.__send_command(wakeup_and_blind)
        self.__send_command(adc_res)

        self.__set_bias_current(self.calib_params['BIAS_calib'])
        self.__set_clock_speed(self.calib_params['CLK_calib'])
        self.__set_cm_current(self.calib_params['BPA_calib'])

        self.__send_command(pull_ups)

    def close(self):
        sleep = self.__generate_command(0x01, 0x00)
        self.__send_command(sleep)

    def get_calib_params(self):
        return self.calib_params
    
    def get_ondemand_frame(self):
        if self.frame_cnt % 25 == 0:
            measure_vdd = True
        else:
            measure_vdd = False

        if self.ts is not None:
            if self.frame_cnt % 10 == 0:
                self.__ask_for_elec_offset()
                self.__wait_for_data()
                top_data, bottom_data = self.__read_top_and_bottom_blocks()
                self.__ask_for_pixel_data(0, measure_vdd=measure_vdd)
                self.elec_offset = self.__format_elec_offset_data(top_data, bottom_data)
            else:
                self.__ask_for_pixel_data(0, measure_vdd=measure_vdd)

            for block in range(0, 4):
                self.__wait_for_data()
                top_data, bottom_data = self.__read_top_and_bottom_blocks()
                if block < 3:
                    self.__ask_for_pixel_data(block+1, measure_vdd=measure_vdd)
                self.__format_pixel_data(top_data, bottom_data, block)
            self.ts = int(time.time()*1000)
            if measure_vdd:
                self.data_vdd = copy.copy(self.header_values)
            else:
                self.data_ptats = copy.copy(self.header_values)
            self.frame_cnt += 1
        else:
            self.__first_read()

        pixel_values = np.reshape(copy.copy(self.pixel_values), (32, 32))
        return pixel_values, self.ts, copy.copy(self.data_ptats), copy.copy(self.data_vdd), copy.copy(self.elec_offset)
    
    def get_continuous_frame(self):
        ### This is similar to get_ondemand_frame(), however it will request the next frame block 0
        ### from the sensor before returning. When called again it will not ask for block 0, it will
        ### read the block then ask for block 1. We assume get_continuous_frame() is called inside a
        ### tight loop.
        if self.frame_cnt % 25 == 0:
            measure_vdd = True
        else:
            measure_vdd = False

        if self.ts is not None:
            if self.frame_cnt % 10 == 0:
                self.__wait_for_data()
                top_data, bottom_data = self.__read_top_and_bottom_blocks()
                self.__ask_for_pixel_data(0, measure_vdd=measure_vdd)
                self.elec_offset = self.__format_elec_offset_data(top_data, bottom_data)

            for block in range(0, 4):
                self.__wait_for_data()
                top_data, bottom_data = self.__read_top_and_bottom_blocks()
                if block < 3:
                    self.__ask_for_pixel_data(block+1, measure_vdd=measure_vdd)
                else:
                    # ask data for next frame
                    if (self.frame_cnt + 1) % 10 == 0:
                        self.__ask_for_elec_offset()
                    else:
                        if (self.frame_cnt + 1) % 25 == 0:
                            next_measure_vdd = True
                        else:
                            next_measure_vdd = False
                        self.__ask_for_pixel_data(0, measure_vdd=next_measure_vdd)
                self.__format_pixel_data(top_data, bottom_data, block)
            self.ts = int(time.time()*1000)
            if measure_vdd:
                self.data_vdd = copy.copy(self.header_values)
            else:
                self.data_ptats = copy.copy(self.header_values)
        else:
            self.__first_read()

        pixel_values = np.reshape(copy.copy(self.pixel_values), (32, 32))
        return pixel_values, self.ts, copy.copy(self.data_ptats), copy.copy(self.data_vdd), copy.copy(self.elec_offset)

    def __set_bias_current(self, bias):
        if bias > 31:
            bias = 31
        if bias < 0:
            bias = 0

        bias = int(bias)

        bias_top = self.__generate_command(0x04, bias)
        bias_bottom = self.__generate_command(0x05, bias)

        self.__send_command(bias_top)
        self.__send_command(bias_bottom)

    def __set_clock_speed(self, clk):
        if clk > 63:
            clk = 63
        if clk < 0:
            clk = 0

        clk = int(clk)

        clk_speed = self.__generate_command(0x06, clk)

        self.__send_command(clk_speed)

    def __set_cm_current(self, cm):
        if cm > 31:
            cm = 31
        if cm < 0:
            cm = 0

        cm = int(cm)

        cm_top = self.__generate_command(0x07, cm)
        cm_bottom = self.__generate_command(0x08, cm)

        self.__send_command(cm_top)
        self.__send_command(cm_bottom)

    def __get_eeprom(self, eeprom_address=0x50):
        # Two separate I2C transfers in case the buffer size is small
        q1 = [I2C.Message([0x00, 0x00]), I2C.Message([0x00]*4000, read=True)]
        q2 = [I2C.Message([0x0f, 0xa0]), I2C.Message([0x00]*4000, read=True)]
        self.i2c.transfer(eeprom_address, q1)
        self.i2c.transfer(eeprom_address, q2)
        return np.array(q1[1].data + q2[1].data)

    def __extract_eeprom_parameters(self, eeprom):
        calib_params = dict()

        calib_params['MBIT_calib'] = eeprom[0x001A]
        calib_params['BIAS_calib'] = eeprom[0x001B]
        calib_params['CLK_calib'] = eeprom[0x001C]
        calib_params['BPA_calib'] = eeprom[0x001D]
        calib_params['PU_calib'] = eeprom[0x001E]

        calib_params['TN'] = eeprom[0x000B] + (eeprom[0x000C]<<8)

        VddCompGrad =   eeprom[0x0340:0x0540:2] + (eeprom[0x0341:0x0540:2] << 8)
        VddCompGrad = [tg - 65536 if tg >= 32768 else tg for tg in VddCompGrad]
        topBlocks = np.reshape(VddCompGrad[0:128], (4, 32))
        bottomBlocks = np.reshape(VddCompGrad[128:], (4, 32))
        VddCompGrad = np.concatenate((topBlocks, topBlocks, topBlocks, topBlocks, bottomBlocks, bottomBlocks, bottomBlocks, bottomBlocks), axis=0)
        VddCompGrad[16:,:] = np.flipud(VddCompGrad[16:,:])
        calib_params['VddCompGrad'] = VddCompGrad

        VddCompOff =   eeprom[0x0540:0x0740:2] + (eeprom[0x0541:0x0740:2] << 8)
        VddCompOff = [tg - 65536 if tg >= 32768 else tg for tg in VddCompOff]
        topBlocks = np.reshape(VddCompOff[0:128], (4, 32))
        bottomBlocks = np.reshape(VddCompOff[128:], (4, 32))
        VddCompOff = np.concatenate((topBlocks, topBlocks, topBlocks, topBlocks, bottomBlocks, bottomBlocks, bottomBlocks, bottomBlocks), axis=0)
        VddCompOff[16:,:] = np.flipud(VddCompOff[16:,:])
        calib_params['VddCompOff'] = VddCompOff

        ThGrad =   eeprom[0x0740:0x0F40:2] + (eeprom[0x0741:0x0F40:2] << 8)
        ThGrad = [tg - 65536 if tg >= 32768 else tg for tg in ThGrad]
        ThGrad = np.reshape(ThGrad, (32, 32))
        ThGrad[16:,:] = np.flipud(ThGrad[16:,:])
        calib_params['ThGrad'] = ThGrad

        ThOffset = eeprom[0x0F40:0x1740:2] + (eeprom[0x0F41:0x1740:2] << 8)
        ThOffset = [tg - 65536 if tg >= 32768 else tg for tg in ThOffset]
        ThOffset = np.reshape(ThOffset, (32, 32))
        ThOffset[16:,:] = np.flipud(ThOffset[16:,:])
        calib_params['ThOffset'] = ThOffset

        P = eeprom[0x1740::2] + (eeprom[0x1741::2] << 8)
        P = np.reshape(P, (32, 32))
        P[16:, :] = np.flipud(P[16:,:])
        calib_params['P'] = P

        epsilon = float(eeprom[0x000D])
        GlobalGain = eeprom[0x0055] + (eeprom[0x0056] << 8)
        Pmin = eeprom[0x0000:0x0004]
        Pmax = eeprom[0x0004:0x0008]
        Pmin = struct.unpack('f', reduce(lambda a,b: a+b, [chr(p) for p in Pmin]))[0]
        Pmax = struct.unpack('f', reduce(lambda a,b: a+b, [chr(p) for p in Pmax]))[0]
        calib_params['PixC'] = (P * (Pmax - Pmin) / 65535. + Pmin) * (epsilon / 100) * float(GlobalGain) / 10000

        calib_params['gradScale'] = eeprom[0x0008]

        PTATgradient = eeprom[0x0034:0x0038]
        calib_params['PTATgradient'] = struct.unpack('f', reduce(lambda a,b: a+b, [chr(p) for p in PTATgradient]))[0]
        PTAToffset = eeprom[0x0038:0x003c]
        calib_params['PTAToffset'] = struct.unpack('f', reduce(lambda a,b: a+b, [chr(p) for p in PTAToffset]))[0]

        calib_params['VddScGrad'] = eeprom[0x004E]
        calib_params['VddScOff'] = eeprom[0x004F]

        calib_params['PTATTh1'] = eeprom[0x003C] + (eeprom[0x003D]<<8)
        calib_params['PTATTh2'] = eeprom[0x003E] + (eeprom[0x003F]<<8)

        calib_params['VDDTh1'] = eeprom[0x0026] + (eeprom[0x0027]<<8)
        calib_params['VDDTh2'] = eeprom[0x0028] + (eeprom[0x0029]<<8)

        return calib_params
    
    def __wait_for_data(self):
        query = [I2C.Message([0x02]), I2C.Message([0x00], read=True)]
        done = False
        while not done:
            self.i2c.transfer(self.address, query)
            if not (query[1].data[0]%2 == 1):
                time.sleep(0.005)
            else:
                done = True
    
    def __first_read(self):
        # on the first read we are going to:
        # read elec offsets
        # read one frame with vdd measurements
        # read one frame with ptat measurements
        self.__ask_for_elec_offset()
        self.__wait_for_data()
        top_data, bottom_data = self.__read_top_and_bottom_blocks()
        self.__ask_for_pixel_data(0, measure_vdd=True)
        self.elec_offset = self.__format_elec_offset_data(top_data, bottom_data)

        measure_vdd = True
        for block in range(0, 4):
            self.__wait_for_data()
            top_data, bottom_data = self.__read_top_and_bottom_blocks()
            if block == 3:
                measure_vdd = False
            self.__ask_for_pixel_data((block+1)%4, measure_vdd=measure_vdd)
            # format block=block data, with vdd header
            self.__format_pixel_data(top_data, bottom_data, block)
        self.data_vdd = copy.copy(self.header_values)

        measure_vdd = False
        for block in range(0, 4):
            self.__wait_for_data()
            top_data, bottom_data = self.__read_top_and_bottom_blocks()
            self.__ask_for_pixel_data((block+1)%4, measure_vdd=measure_vdd)
            # format block 0 data, with vdd header
            self.__format_pixel_data(top_data, bottom_data, block)
        self.ts = int(time.time()*1000)
        self.data_ptats = copy.copy(self.header_values)

        self.frame_cnt = 1


    def __format_pixel_data(self, top_data, bottom_data, block):
        top_data = top_data[1::2] + (top_data[0::2] << 8)
        bottom_data = bottom_data[1::2] + (bottom_data[0::2] << 8)

        self.pixel_values[(0+block*128):(128+block*128)] = top_data[1:]
        # bottom data is in a weird shape
        self.pixel_values[(992-block*128):(1024-block*128)] = bottom_data[1:33]
        self.pixel_values[(960-block*128):(992-block*128)] = bottom_data[33:65]
        self.pixel_values[(928-block*128):(960-block*128)] = bottom_data[65:97]
        self.pixel_values[(896-block*128):(928-block*128)] = bottom_data[97:]

        self.header_values[block] = top_data[0]
        self.header_values[7-block] = bottom_data[0]
    
    def __ask_for_elec_offset(self):
        self.__send_command(self.__generate_expose_block_command(block=None, blind=True), wait=False)
    
    def __ask_for_pixel_data(self, block, measure_vdd):
        self.__send_command(self.__generate_expose_block_command(block, blind=False, vdd=measure_vdd), wait=False)
    
    def __read_top_and_bottom_blocks(self):
        read_block = [I2C.Message([0x0A]), I2C.Message([0x00]*258, read=True)]
        self.i2c.transfer(self.address, read_block)
        top_data = np.array(copy.copy(read_block[1].data))

        read_block = [I2C.Message([0x0B]), I2C.Message([0x00]*258, read=True)]
        self.i2c.transfer(self.address, read_block)
        bottom_data = np.array(copy.copy(read_block[1].data))

        return top_data, bottom_data
    
    def __format_elec_offset_data(self, top_data, bottom_data):
        top_data = top_data[1::2] + (top_data[0::2] << 8)
        bottom_data = bottom_data[1::2] + (bottom_data[0::2] << 8)

        top_block = np.reshape(top_data[1:], (4, 32))
        bottom_block = np.reshape(bottom_data[1:], (4, 32))

        bottom_block = np.flipud(bottom_block)

        elec_offset = np.concatenate((top_block, top_block, top_block, top_block, bottom_block, bottom_block, bottom_block, bottom_block), axis=0)
        elec_offset = elec_offset.astype(np.float)

        return elec_offset

    def __generate_command(self, register, value):
        return [I2C.Message([register, value])]

    def __generate_expose_block_command(self, block, blind=False, vdd=False):
        if blind:
            return self.__generate_command(0x01, 0x09 + 0x02)
        else:
            vdd_meas = 0
            if vdd:
                vdd_meas = 0x04
            return self.__generate_command(0x01, 0x09 + (block << self.blockshift) + vdd_meas)

    def __send_command(self, cmd, wait=True):
        self.i2c.transfer(self.address, cmd)
        if wait:
            time.sleep(0.005) # sleep for 5 ms


