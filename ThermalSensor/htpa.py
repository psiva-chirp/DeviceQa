from periphery import I2C
import time
import numpy as np
import copy
import struct
from scipy import interpolate

class HTPA:
    def __init__(self, temp_table=None, address=0x1A, i2c_bus="/dev/i2c-5"):
        self.address = address
        self.i2c = I2C(i2c_bus)

        self.ptats = None 
        self.vdd = None

        self.frame_cnt = 0

        self.blockshift = 4

        print ("Grabbing EEPROM data")

        eeprom = self.get_eeprom()
        self.extract_eeprom_parameters(eeprom)
        self.eeprom = eeprom
        print(eeprom)

        self.temp_table = None
        for table in temp_table:
            if table['TABLENUMBER'] == self.TN:
                self.temp_table = table
        if temp_table is not None:
            self.temp_interp_f = interpolate.interp2d(self.temp_table['XTATemps'], self.temp_table['YADValues'], self.temp_table['TempTable'], kind='linear')
        else:
            self.temp_interp_f = None
            print('Could not find Table Number %d in temp_table provided' % self.TN)

        print("Initializing capture settings with stored calibration data")
        wakeup_and_blind = self.generate_command(0x01, 0x01) # wake up the device
        adc_res = self.generate_command(0x03, self.MBIT_calib) # set ADC resolution to 16 bits
        pull_ups = self.generate_command(0x09, self.PU_calib)

        self.send_command(wakeup_and_blind)
        self.send_command(adc_res)

        self.set_bias_current(self.BIAS_calib)
        self.set_clock_speed(self.CLK_calib)
        self.set_cm_current(self.BPA_calib)

        self.send_command(pull_ups)

        # initialize offset to zero
        self.elecOffset = np.zeros((32, 32))

    def set_bias_current(self, bias):
        if bias > 31:
            bias = 31
        if bias < 0:
            bias = 0

        bias = int(bias)

        bias_top = self.generate_command(0x04, bias)
        bias_bottom = self.generate_command(0x05, bias)

        self.send_command(bias_top)
        self.send_command(bias_bottom)

    def set_clock_speed(self, clk):
        if clk > 63:
            clk = 63
        if clk < 0:
            clk = 0

        clk = int(clk)

        clk_speed = self.generate_command(0x06, clk)

        self.send_command(clk_speed)

    def set_cm_current(self, cm):
        if cm > 31:
            cm = 31
        if cm < 0:
            cm = 0

        cm = int(cm)

        cm_top = self.generate_command(0x07, cm)
        cm_bottom = self.generate_command(0x08, cm)

        self.send_command(cm_top)
        self.send_command(cm_bottom)

    def get_eeprom(self, eeprom_address=0x50):
        # Two separate I2C transfers in case the buffer size is small
        q1 = [I2C.Message([0x00, 0x00]), I2C.Message([0x00]*4000, read=True)]
        q2 = [I2C.Message([0x0f, 0xa0]), I2C.Message([0x00]*4000, read=True)]
        self.i2c.transfer(eeprom_address, q1)
        self.i2c.transfer(eeprom_address, q2)
        return np.array(q1[1].data + q2[1].data)

    def extract_eeprom_parameters(self, eeprom):
        self.MBIT_calib = eeprom[0x001A]
        self.BIAS_calib = eeprom[0x001B]
        self.CLK_calib = eeprom[0x001C]
        self.BPA_calib = eeprom[0x001D]
        self.PU_calib = eeprom[0x001E]
        print('EEPROM')
        print('MBIT(calib) %d', self.MBIT_calib)
        print('BIAS(calib) %d', self.BIAS_calib)
        print('CLK(calib) %d', self.CLK_calib)
        print('BPA(calib) %d', self.BPA_calib)
        print('PU(calib) %d', self.PU_calib)

        '''
        print('MBIT(user) %d', eeprom[0x0060])
        print('BIAS(user) %d', eeprom[0x0061])
        print('CLK(user) %d', eeprom[0x0062])
        print('BPA(user) %d', eeprom[0x0063])
        print('PU(user) %d', eeprom[0x0064])
        '''

        self.TN = eeprom[0x000B] + (eeprom[0x000C]<<8)
        print('TN %d' % self.TN)

        print('VddCompGrad')
        VddCompGrad =   eeprom[0x0340:0x0540:2] + (eeprom[0x0341:0x0540:2] << 8)
        print(VddCompGrad.shape)
        print(np.min(VddCompGrad))
        print(np.max(VddCompGrad))
        VddCompGrad = [tg - 65536 if tg >= 32768 else tg for tg in VddCompGrad]
        print(np.min(VddCompGrad))
        print(np.max(VddCompGrad))
        topBlocks = np.reshape(VddCompGrad[0:128], (4, 32))
        bottomBlocks = np.reshape(VddCompGrad[128:], (4, 32))
        VddCompGrad = np.concatenate((topBlocks, topBlocks, topBlocks, topBlocks, bottomBlocks, bottomBlocks, bottomBlocks, bottomBlocks), axis=0)
        print(VddCompGrad.shape)
        VddCompGrad[16:,:] = np.flipud(VddCompGrad[16:,:])
        self.VddCompGrad = VddCompGrad

        print('VddCompOff')
        VddCompOff =   eeprom[0x0540:0x0740:2] + (eeprom[0x0541:0x0740:2] << 8)
        print(VddCompOff.shape)
        print(np.min(VddCompOff))
        print(np.max(VddCompOff))
        VddCompOff = [tg - 65536 if tg >= 32768 else tg for tg in VddCompOff]
        print(np.min(VddCompOff))
        print(np.max(VddCompOff))
        topBlocks = np.reshape(VddCompOff[0:128], (4, 32))
        bottomBlocks = np.reshape(VddCompOff[128:], (4, 32))
        VddCompOff = np.concatenate((topBlocks, topBlocks, topBlocks, topBlocks, bottomBlocks, bottomBlocks, bottomBlocks, bottomBlocks), axis=0)
        print(VddCompOff.shape)
        VddCompOff[16:,:] = np.flipud(VddCompOff[16:,:])
        self.VddCompOff = VddCompOff

        print('ThGrad')
        ThGrad =   eeprom[0x0740:0x0F40:2] + (eeprom[0x0741:0x0F40:2] << 8)
        print(np.min(ThGrad))
        print(np.max(ThGrad))
        ThGrad = [tg - 65536 if tg >= 32768 else tg for tg in ThGrad]
        print(np.min(ThGrad))
        print(np.max(ThGrad))
        ThGrad = np.reshape(ThGrad, (32, 32))
        ThGrad[16:,:] = np.flipud(ThGrad[16:,:])
        self.ThGrad = ThGrad

        print('ThOffset')
        ThOffset = eeprom[0x0F40:0x1740:2] + (eeprom[0x0F41:0x1740:2] << 8)
        print(np.min(ThOffset))
        print(np.max(ThOffset))
        ThOffset = [tg - 65536 if tg >= 32768 else tg for tg in ThOffset]
        print(np.min(ThOffset))
        print(np.max(ThOffset))
        ThOffset = np.reshape(ThOffset, (32, 32))
        ThOffset[16:,:] = np.flipud(ThOffset[16:,:])
        self.ThOffset = ThOffset

        P = eeprom[0x1740::2] + (eeprom[0x1741::2] << 8)
        P = np.reshape(P, (32, 32))
        P[16:, :] = np.flipud(P[16:,:])
        self.P = P

        epsilon = float(eeprom[0x000D])
        GlobalGain = eeprom[0x0055] + (eeprom[0x0056] << 8)
        Pmin = eeprom[0x0000:0x0004]
        Pmax = eeprom[0x0004:0x0008]
        Pmin = struct.unpack('f', reduce(lambda a,b: a+b, [chr(p) for p in Pmin]))[0]
        Pmax = struct.unpack('f', reduce(lambda a,b: a+b, [chr(p) for p in Pmax]))[0]
        self.PixC = (P * (Pmax - Pmin) / 65535. + Pmin) * (epsilon / 100) * float(GlobalGain) / 10000

        self.gradScale = eeprom[0x0008]

        PTATgradient = eeprom[0x0034:0x0038]
        self.PTATgradient = struct.unpack('f', reduce(lambda a,b: a+b, [chr(p) for p in PTATgradient]))[0]
        PTAToffset = eeprom[0x0038:0x003c]
        self.PTAToffset = struct.unpack('f', reduce(lambda a,b: a+b, [chr(p) for p in PTAToffset]))[0]
        print('PTATgradient %f' % self.PTATgradient)
        print('PTAToffset %f' % self.PTAToffset)

        self.VddScGrad = eeprom[0x004E]
        self.VddScOff = eeprom[0x004F]

        self.PTATTh1 = eeprom[0x003C] + (eeprom[0x003D]<<8)
        self.PTATTh2 = eeprom[0x003E] + (eeprom[0x003F]<<8)

        self.VDDTh1 = eeprom[0x0026] + (eeprom[0x0027]<<8)
        self.VDDTh2 = eeprom[0x0028] + (eeprom[0x0029]<<8)

    def get_ambient_temperature_dK(self):
        Ta = np.mean(self.ptats) * self.PTATgradient + self.PTAToffset
        return Ta

    def temperature_compensation(self, im):
        comp = ((self.ThGrad*np.mean(self.ptats)) / pow(2, self.gradScale)) + self.ThOffset

        #print('v00, thgrad, ptatav, gradscale, thoffset')
        #print(im[16,16])
        #print(self.ThGrad[16,16])
        #print(np.mean(self.ptats))
        #print(self.gradScale)
        #print(self.ThOffset[16,16])
        #print(comp[16,16])

        Vcomp = im - comp
        #print(Vcomp[16,16])

        return Vcomp
    
    def vdd_compensation(self, im):
        if self.vdd is None:
            return im
        vdd_av = np.mean(self.vdd)
        ptat_av = np.mean(self.ptats)

        numerator1 = ((self.VddCompGrad*ptat_av)/pow(2, self.VddScGrad) + self.VddCompOff)
        numerator2 = vdd_av - self.VDDTh1 - ((self.VDDTh2-self.VDDTh1)/(self.PTATTh2-self.PTATTh1))*(ptat_av - self.PTATTh1)
        comp = numerator1*numerator2/(pow(2, self.VddScOff))
        im = im - comp
        return im

    def elec_offset_compensation(self, im):
        return im - self.elecOffset

    def sensitivity_compensation(self, im):
        PCSCALEVAL = 100000000
        im = (im* PCSCALEVAL)/self.PixC
        return im

    def capture_elec_offset(self):
        self.send_command(self.generate_expose_block_command(block=None, blind=True), wait=False)

        query = [I2C.Message([0x02]), I2C.Message([0x00], read=True)]

        done = False

        while not done:
            self.i2c.transfer(self.address, query)

            if not (query[1].data[0]%2 == 1):
                #print("Not ready, received " + str(query[1].data[0]) + ", expected " + str(1))
                time.sleep(0.005)
            else:
                done = True

        read_block = [I2C.Message([0x0A]), I2C.Message([0x00]*258, read=True)]
        self.i2c.transfer(self.address, read_block)
        top_data = np.array(copy.copy(read_block[1].data))

        read_block = [I2C.Message([0x0B]), I2C.Message([0x00]*258, read=True)]
        self.i2c.transfer(self.address, read_block)
        bottom_data = np.array(copy.copy(read_block[1].data))
        
        top_data = top_data[1::2] + (top_data[0::2] << 8)
        bottom_data = bottom_data[1::2] + (bottom_data[0::2] << 8)

        top_block = np.reshape(top_data[1:], (4, 32))
        bottom_block = np.reshape(bottom_data[1:], (4, 32))

        bottom_block = np.flipud(bottom_block)

        elecOffset = np.concatenate((top_block, top_block, top_block, top_block, bottom_block, bottom_block, bottom_block, bottom_block), axis=0)
        elecOffset = elecOffset.astype(np.float)
        
        self.elecOffset = elecOffset
    
    def get_image(self):
        self.capture_elec_offset()
        pixel_values = self.capture_image()
        im = self.elec_offset_compensation(pixel_values)
        im = self.temperature_compensation(im)
        im = self.vdd_compensation(im)
        im = self.sensitivity_compensation(im)
        Ta = self.get_ambient_temperature_dK()
        if self.temp_table is not None:
            left_insert_idx = np.searchsorted(self.temp_table['XTATemps'], Ta)
            col_start_end = [left_insert_idx-1, left_insert_idx]
            if col_start_end[1] >= len(self.temp_table['XTATemps']):
                col_start_end[1] = -1
            if col_start_end[0] == -1 or col_start_end[1] == -1:
                # can't do conversion
                return im
            im_temp = self.temp_interp_f(np.ones((im.shape[0],im.shape[1]))*Ta, im + self.temp_table['TABLEOFFSET'])
            print('here')
            '''
            temp_table_data = np.asarray(self.temp_table['TempTable'])
            Ta_alpha = (Ta - self.temp_table['XTATemps'][col_start_end[0]])/(self.temp_table['XTATemps'][col_start_end[1]] - self.temp_table['XTATemps'][col_start_end[0]])
            for r in range(0, im.shape[0]):
                for c in range(0, im.shape[1]):
                    row_start_end = [-1, -1]
                    row_start_end[0] = int((im[r,c] + self.temp_table['TABLEOFFSET'])/self.temp_table['ADEQUIDISTANCE'])
                    row_start_end[1] = row_start_end[0] + 1
                    va_alpha = (im[r,c] - self.temp_table['YADValues'][row_start_end[0]])/(self.temp_table['YADValues'][row_start_end[1]] - self.temp_table['YADValues'][row_start_end[0]])
            '''
        return im


    def capture_image(self, blind=False):
        pixel_values = np.zeros(1024)
        ptats = np.zeros(8)

        self.frame_cnt += 1
        if self.frame_cnt % 10 == 0:
            get_vdd = True
        else:
            get_vdd = False

        for block in range(4):
            #print("Exposing block " + str(block))
            self.send_command(self.generate_expose_block_command(block, blind=blind, vdd=get_vdd), wait=False)

            query = [I2C.Message([0x02]), I2C.Message([0x00], read=True)]
            expected = 1 + (block << 4)

            done = False

            while not done:
                self.i2c.transfer(self.address, query)

                if not (query[1].data[0]%2 == 1):
                    #print("Not ready, received " + str(query[1].data[0]) + ", expected " + str(expected))
                    time.sleep(0.005)
                else:
                    done = True

            read_block = [I2C.Message([0x0A]), I2C.Message([0x00]*258, read=True)]
            self.i2c.transfer(self.address, read_block)
            top_data = np.array(copy.copy(read_block[1].data))

            read_block = [I2C.Message([0x0B]), I2C.Message([0x00]*258, read=True)]
            self.i2c.transfer(self.address, read_block)
            bottom_data = np.array(copy.copy(read_block[1].data))

            top_data = top_data[1::2] + (top_data[0::2] << 8)
            bottom_data = bottom_data[1::2] + (bottom_data[0::2] << 8)

            pixel_values[(0+block*128):(128+block*128)] = top_data[1:]
            # bottom data is in a weird shape
            pixel_values[(992-block*128):(1024-block*128)] = bottom_data[1:33]
            pixel_values[(960-block*128):(992-block*128)] = bottom_data[33:65]
            pixel_values[(928-block*128):(960-block*128)] = bottom_data[65:97]
            pixel_values[(896-block*128):(928-block*128)] = bottom_data[97:]

            ptats[block] = top_data[0]
            ptats[7-block] = bottom_data[0]

        pixel_values = np.reshape(pixel_values, (32, 32))
        if get_vdd:
            self.vdd = ptats
        else:
            self.ptats = ptats

        return pixel_values

    def generate_command(self, register, value):
        return [I2C.Message([register, value])]

    def generate_expose_block_command(self, block, blind=False, vdd=False):
        if blind:
            return self.generate_command(0x01, 0x09 + 0x02)
        else:
            vdd_meas = 0
            if vdd:
                vdd_meas = 0x04
            return self.generate_command(0x01, 0x09 + (block << self.blockshift) + vdd_meas)

    def send_command(self, cmd, wait=True):
        self.i2c.transfer(self.address, cmd)
        if wait:
            time.sleep(0.005) # sleep for 5 ms

    def close(self):
        sleep = self.generate_command(0x01, 0x00)
        self.send_command(sleep)


class HTPAV1:
    def __init__(self, address, revision="2018"):
        self.address = address
        self.i2c = I2C("/dev/i2c-5")

        if revision == "2018":
            self.blockshift = 4
        else:
            self.blockshift = 2

        wakeup_and_blind = self.generate_command(0x01, 0x01) # wake up the device
        adc_res = self.generate_command(0x03, 0x0C) # set ADC resolution to 16 bits
        pull_ups = self.generate_command(0x09, 0x88)

        print("Initializing capture settings")

        self.send_command(wakeup_and_blind)
        self.send_command(adc_res)
        self.send_command(pull_ups)

        self.set_bias_current(0x05)
        self.set_clock_speed(0x3F)
        self.set_cm_current(0x0C)

        print ("Grabbing EEPROM data")

        eeprom = self.get_eeprom()
        self.extract_eeprom_parameters(eeprom)
        self.eeprom = eeprom
        print(eeprom)

        # initialize offset to zero
        self.offset = np.zeros((32, 32))

    def set_bias_current(self, bias):
        if bias > 31:
            bias = 31
        if bias < 0:
            bias = 0

        bias = int(bias)

        bias_top = self.generate_command(0x04, bias)
        bias_bottom = self.generate_command(0x05, bias)

        self.send_command(bias_top)
        self.send_command(bias_bottom)

    def set_clock_speed(self, clk):
        if clk > 63:
            clk = 63
        if clk < 0:
            clk = 0

        clk = int(clk)

        clk_speed = self.generate_command(0x06, clk)

        self.send_command(clk_speed)

    def set_cm_current(self, cm):
        if cm > 31:
            cm = 31
        if cm < 0:
            cm = 0

        cm = int(cm)

        cm_top = self.generate_command(0x07, cm)
        cm_bottom = self.generate_command(0x08, cm)

        self.send_command(cm_top)
        self.send_command(cm_bottom)

    def get_eeprom(self, eeprom_address=0x50):
        # Two separate I2C transfers in case the buffer size is small
        q1 = [I2C.Message([0x00, 0x00]), I2C.Message([0x00]*4000, read=True)]
        q2 = [I2C.Message([0x0f, 0xa0]), I2C.Message([0x00]*4000, read=True)]
        self.i2c.transfer(eeprom_address, q1)
        self.i2c.transfer(eeprom_address, q2)
        return np.array(q1[1].data + q2[1].data)

    def extract_eeprom_parameters(self, eeprom):
        self.VddComp = eeprom[0x0540:0x0740:2] + (eeprom[0x0541:0x0740:2] << 8)

        ThGrad =   eeprom[0x0740:0x0F40:2] + (eeprom[0x0741:0x0F40:2] << 8)
        ThGrad = [tg - 65536 if tg >= 32768 else tg for tg in ThGrad]
        ThGrad = np.reshape(ThGrad, (32, 32))
        ThGrad[16:,:] = np.flipud(ThGrad[16:,:])
        self.ThGrad = ThGrad

        ThOffset = eeprom[0x0F40:0x1740:2] + (eeprom[0x0F41:0x1740:2] << 8)
        ThOffset = np.reshape(ThOffset, (32, 32))
        ThOffset[16:,:] = np.flipud(ThOffset[16:,:])
        self.ThOffset = ThOffset

        P = eeprom[0x1740::2] + (eeprom[0x1741::2] << 8)
        P = np.reshape(P, (32, 32))
        P[16:, :] = np.flipud(P[16:,:])
        self.P = P

        epsilon = float(eeprom[0x000D])
        GlobalGain = eeprom[0x0055] + (eeprom[0x0056] << 8)
        Pmin = eeprom[0x0000:0x0004]
        Pmax = eeprom[0x0004:0x0008]
        Pmin = struct.unpack('f', reduce(lambda a,b: a+b, [chr(p) for p in Pmin]))[0]
        Pmax = struct.unpack('f', reduce(lambda a,b: a+b, [chr(p) for p in Pmax]))[0]
        self.PixC = (P * (Pmax - Pmin) / 65535. + Pmin) * (epsilon / 100) * float(GlobalGain) / 100

        self.gradScale = eeprom[0x0008]
        self.VddCalib = eeprom[0x0046] + (eeprom[0x0047] << 8)
        self.Vdd = 3280.0
        self.VddScaling = eeprom[0x004E]

        PTATgradient = eeprom[0x0034:0x0038]
        self.PTATgradient = struct.unpack('f', reduce(lambda a,b: a+b, [chr(p) for p in PTATgradient]))[0]
        PTAToffset = eeprom[0x0038:0x003c]
        self.PTAToffset = struct.unpack('f', reduce(lambda a,b: a+b, [chr(p) for p in PTAToffset]))[0]

    def temperature_compensation(self, im, ptat):
        comp = np.zeros((32,32))
        
        Ta = np.mean(ptat) * self.PTATgradient + self.PTAToffset
        #     temperature compensated voltage
        comp = ((self.ThGrad * Ta) / pow(2, self.gradScale)) + self.ThOffset

        Vcomp = np.reshape(im,(32, 32)) - comp
        return Vcomp

    def offset_compensation(self, im):
        return im - self.offset

    def sensitivity_compensation(self, im):
        return im/self.PixC

    def measure_observed_offset(self):
        print("Measuring observed offsets")
        print("   Camera should be against uniform temperature surface")
        mean_offset = np.zeros((32, 32))

        for i in range(10):
            print("    frame " + str(i))
            (p, pt) = self.capture_image()
            im = self.temperature_compensation(p, pt)
            mean_offset += im/10.0

        self.offset = mean_offset

    def measure_electrical_offset(self):
        (offsets, ptats) = self.capture_image(blind=True)
        self.offset = self.temperature_compensation(offsets, ptats)

    def capture_image(self, blind=False):
        pixel_values = np.zeros(1024)
        ptats = np.zeros(8)

        for block in range(4):
            #print("Exposing block " + str(block))
            self.send_command(self.generate_expose_block_command(block, blind=blind), wait=False)

            query = [I2C.Message([0x02]), I2C.Message([0x00], read=True)]
            expected = 1 + (block << 4)

            done = False

            while not done:
                self.i2c.transfer(self.address, query)

                if not (query[1].data[0] == expected):
                    #print("Not ready, received " + str(query[1].data[0]) + ", expected " + str(expected))
                    time.sleep(0.005)
                else:
                    done = True

            read_block = [I2C.Message([0x0A]), I2C.Message([0x00]*258, read=True)]
            self.i2c.transfer(self.address, read_block)
            top_data = np.array(copy.copy(read_block[1].data))

            read_block = [I2C.Message([0x0B]), I2C.Message([0x00]*258, read=True)]
            self.i2c.transfer(self.address, read_block)
            bottom_data = np.array(copy.copy(read_block[1].data))

            top_data = top_data[1::2] + (top_data[0::2] << 8)
            bottom_data = bottom_data[1::2] + (bottom_data[0::2] << 8)

            pixel_values[(0+block*128):(128+block*128)] = top_data[1:]
            # bottom data is in a weird shape
            pixel_values[(992-block*128):(1024-block*128)] = bottom_data[1:33]
            pixel_values[(960-block*128):(992-block*128)] = bottom_data[33:65]
            pixel_values[(928-block*128):(960-block*128)] = bottom_data[65:97]
            pixel_values[(896-block*128):(928-block*128)] = bottom_data[97:]

            ptats[block] = top_data[0]
            ptats[7-block] = bottom_data[0]

        pixel_values = np.reshape(pixel_values, (32, 32))

        return (pixel_values, ptats)

    def generate_command(self, register, value):
        return [I2C.Message([register, value])]

    def generate_expose_block_command(self, block, blind=False):
        if blind:
            return self.generate_command(0x01, 0x09 + (block << self.blockshift) + 0x02)
        else:
            return self.generate_command(0x01, 0x09 + (block << self.blockshift))

    def send_command(self, cmd, wait=True):
        self.i2c.transfer(self.address, cmd)
        if wait:
            time.sleep(0.005) # sleep for 5 ms

    def close(self):
        sleep = self.generate_command(0x01, 0x00)
        self.send_command(sleep)
