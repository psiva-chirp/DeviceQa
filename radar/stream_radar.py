import time
import struct
import copy
import numpy as np
import serial

# Constants
MMWDEMO_UART_MSG_DETECTED_POINTS = 1
MMWDEMO_UART_MSG_RANGE_PROFILE = 2
MAGIC_WORD = np.array([2, 1, 4, 3, 6, 5, 8, 7])
MAX_REPEATED_EMPTY_READS = 100

MMWDEMO_OUTPUT_MSG_TYPES = {
    'DETECTED_POINTS': 1,
    'RANGE_PROFILE': 2,
    'NOISE_PROFILE': 3,
    'AZIMUT_STATIC_HEAT_MAP': 4,
    'RANGE_DOPPLER_HEAT_MAP': 5,
    'STATS': 6,
    'DETECTED_POINTS_SIDE_INFO': 7,
    'AZIMUT_ELEVATION_STATIC_HEAT_MAP': 8,
    'MAX': 9
}


class NoDataException(Exception):
    def __init__(self, no_data_iterations):
        self.no_data_iterations = no_data_iterations


class Stream6843AOP:
    def __init__(self, config_str=None, cli_port=None, data_port=None):
        self.cli_port = cli_port
        self.data_port = data_port
        self.config_str = config_str

        self.cli_interface = None
        self.data_interface = None
        self.config_params = None
        self.byte_buffer = np.zeros(2 ** 15, dtype='uint8')
        self.byte_buffer_len = 0
        self.repeated_empty_reads = 0

        self.previous_doppler_sum = None

        self.setup_device(config_str, cli_port, data_port)

    def get_horizontal_antenna_idxs(self):
        return [7, 5, 11, 9]

    def get_vertical_antenna_idxs(self):
        return [2, 3, 6, 7]

    def stop_sensor(self):
        self.cli_interface.write(('sensorStop\n').encode())

    def start_sensor(self):
        self.cli_interface.write(('sensorStart\n').encode())

    def close_ports(self):
        self.cli_interface.close()
        self.data_interface.close()

    def setup_device(self, config_str, cli_port, data_port):
        self.set_ports(cli_port, data_port)
        self.set_serial_config(config_str)

    def resetup_device(self):
        self.setup_device(self.config_str, self.cli_port, self.data_port)

    def set_ports(self, cli_port, data_port):
        self.cli_interface = serial.Serial(cli_port, 115200)
        self.data_interface = serial.Serial(data_port, 921600)

    def send_chunk(self, line):
        n = 11
        chunks = [line[i:i+n] for i in range(0, len(line), n)]
        for c in chunks:
            self.cli_interface.write(c.encode('utf-8'))
            time.sleep(0.03)
        read_buffer = self.cli_interface.read(self.cli_interface.in_waiting)
        time.sleep(0.03)
        print(read_buffer)
    def set_serial_config(self, config_str):
        config_lines = config_str.splitlines()
        for line in config_lines:
            self.send_chunk(line+'\n')
            print("CFG: %s" % line)
            # Yuck
            time.sleep(0.01)
        self.ti_config_params = self.parse_ti_config(config_lines)

    def parse_ti_config(self, config_lines):
        for line in config_lines:
            cfg = line.split(maxsplit=1)
            cfg_key = cfg[0]

            if "profileCfg" == cfg_key:
                cfg_vals = cfg[1].split()
                num_range_bins = int(cfg_vals[9])

            elif "frameCfg" == cfg_key:
                cfg_vals = cfg[1].split()
                chirp_start_idx = int(cfg_vals[0])
                chirp_end_idx = int(cfg_vals[1])
                num_tx_antennas = chirp_end_idx - chirp_start_idx + 1
                num_chirps_per_frame = int(cfg_vals[2])
                frame_time_ms = float(cfg_vals[4])

        config_params = dict()
        config_params['num_rx_antenna'] = 4  # hard coded, IWR6843AOP has 4
        config_params['num_tx_antenna'] = num_tx_antennas
        config_params['fps'] = 1000.0 / frame_time_ms
        config_params['num_chirps_per_frame'] = num_chirps_per_frame
        config_params['num_range_bins'] = num_range_bins

        print("CFG: %s" % config_params)

        return config_params

    def parse_uint16_tlv_data(self, data, tlv_length):
        uint16_data = list()
        for i in range(int(tlv_length / 2)):
            val = struct.unpack('H', data[2 * i:2 * i + 2])
            uint16_data.append(val[0])
        return uint16_data

    def parse_float_tlv_data(self, data, tlv_length):
        float_data = list()
        for i in range(int(tlv_length / 4)):
            val = struct.unpack('f', data[4 * i:4 * i + 4])
            float_data.append(val[0])
        return float_data

    def parse_int32_tlv_data(self, data, tlv_length):
        int32_data = list()
        for i in range(int(tlv_length / 4)):
            val = struct.unpack('i', data[4 * i:4 * i + 4])
            int32_data.append(val[0])
        return int32_data

    def parse_int16_tlv_data(self, data, tlv_length):
        int32_data = list()
        for i in range(int(tlv_length / 2)):
            val = struct.unpack('h', data[2 * i:2 * i + 2])
            int32_data.append(val[0])
        return int32_data

    def parse_azimuth_elevation_range(self, data, tlv_length, config_params):
        num_virtual_antenas = config_params['num_rx_antenna'] * config_params['num_tx_antenna']
        range_data = list()
        for i in range(int(tlv_length / 4)):
            imag = struct.unpack('h', data[4 * i:4 * i + 2])[0]
            real = struct.unpack('h', data[4 * i + 2:4 * i + 4])[0]
            range_data.append(real + imag * 1j)
        antenna_matrix = np.asarray(range_data)
        antenna_matrix = np.reshape(antenna_matrix, (-1, num_virtual_antenas))
        # note antenna order is (TX0, RX0), (TX0, RX1),(TX0, RX2), (TX0, RX3),(TX1, RX0), (TX1, RX1) ..., (TX2, RX2), (TX2, RX3)
        return antenna_matrix

    def parse_range_profile(self, data, tlv_length, config_params):
        data_vals = self.parse_uint16_tlv_data(data, tlv_length)
        tmp = np.asarray(data_vals)
        tmp = tmp.astype(np.float32)
        tmp = np.roll(tmp, int(len(tmp) / 2), axis=0)
        return tmp

    def parse_range_sum(self, data, tlv_length, config_params):
        '''
        num_tx = 2
        num_rx = 2
        num_range = 21
        num_chirp = 32
        #data_vals = self.parse_int32_tlv_data(data, tlv_length)
        data_vals = self.parse_int16_tlv_data(data, tlv_length)
        '''

        num_tx = 1
        num_rx = 1
        num_range = 256
        num_chirp = 1
        #data_vals = self.parse_int32_tlv_data(data, tlv_length)
        data_vals = self.parse_int16_tlv_data(data, tlv_length)

        tmp = np.asarray(data_vals)
        tmp = tmp.astype(np.float32)
        tmp2 = np.zeros((int(tmp.shape[0] / 2),), dtype=np.complex128)
        for i in range(tmp2.shape[0]):
            tmp2[i] = tmp[i * 2 + 0] + tmp[i * 2 + 1] * 1j
        tmp3 = np.zeros((num_chirp, num_range, num_tx * num_rx), dtype=np.complex128)
        range_idx = 0
        rx_idx = 0
        chirp_idx = 0
        tx_idx = 0
        for idx, val in enumerate(tmp2):
            tmp3[chirp_idx, range_idx, tx_idx * num_rx + rx_idx] = val
            range_idx += 1
            if range_idx == num_range:
                range_idx = 0
                rx_idx += 1
            if rx_idx == num_rx:
                rx_idx = 0
                chirp_idx += 1
            if chirp_idx == num_chirp:
                chirp_idx = 0
                tx_idx += 1
            if tx_idx == num_tx:
                tx_idx == 0

        return tmp3

    def parse_range_doppler(self, data, tlv_length, config_params):
        data_vals = self.parse_uint16_tlv_data(data, tlv_length)
        tmp = np.asarray(data_vals)
        tmp = tmp.astype(np.float32)
        tmp = np.reshape(tmp, (config_params['num_range_bins'], config_params['num_chirps_per_frame']))
        tmp /= 256
        tmp = np.roll(tmp, int(config_params['num_chirps_per_frame'] / 2), axis=1)
        return tmp

    def remove_from_buffer(self, num_bytes):
        self.byte_buffer[:self.byte_buffer_len - num_bytes] = self.byte_buffer[num_bytes:self.byte_buffer_len]
        self.byte_buffer[self.byte_buffer_len - num_bytes:] = np.zeros(
            len(self.byte_buffer[self.byte_buffer_len - num_bytes:]), dtype='uint8')
        self.byte_buffer_len = self.byte_buffer_len - num_bytes
        if self.byte_buffer_len < 0:
            self.byte_buffer_len = 0

    def get_packet(self):
        if self.byte_buffer_len <= 16:
            return None

        possible_locs = np.where(self.byte_buffer == MAGIC_WORD[0])[0]

        start_idx = []
        for loc in possible_locs:
            check = self.byte_buffer[loc:loc + 8]
            if np.all(check == MAGIC_WORD):
                start_idx.append(loc)

        if not start_idx:
            return None

        if 0 < start_idx[0] < self.byte_buffer_len:
            self.remove_from_buffer(start_idx[0])

        total_packet_len = struct.unpack('i', self.byte_buffer[12:12 + 4])[0]

        if (self.byte_buffer_len >= total_packet_len) and (self.byte_buffer_len != 0):
            packet = copy.deepcopy(self.byte_buffer[0:total_packet_len])
            self.remove_from_buffer(total_packet_len)
            return packet

        return None

    def get_latest_packet(self):
        packet = None
        cnt = 0
        # Trip wire set at 10
        while cnt < 10:
            next_packet = self.get_packet()
            if next_packet is not None:
                packet = next_packet
                cnt += 1
            else:
                return packet
        raise Exception("Could not get latest radar packet")

    def parse_header(self, packet):
        data_header = dict()

        data_header['magic_number'] = packet[:8]
        packet = packet[8:]
        data_header['version'] = format(struct.unpack('i', packet[0:4])[0], 'x')
        packet = packet[4:]
        data_header['packet_len'] = struct.unpack('i', packet[0:4])[0]
        packet = packet[4:]
        data_header['platform'] = format(struct.unpack('i', packet[0:4])[0], 'x')
        packet = packet[4:]
        data_header['frame_number'] = struct.unpack('i', packet[0:4])[0]
        packet = packet[4:]
        data_header['time_cpu_cycles'] = struct.unpack('i', packet[0:4])[0]
        packet = packet[4:]
        data_header['num_detected_objects'] = struct.unpack('i', packet[0:4])[0]
        packet = packet[4:]
        data_header['num_TLVs'] = struct.unpack('i', packet[0:4])[0]
        packet = packet[4:]
        data_header['sub_frame_number'] = struct.unpack('i', packet[0:4])[0]
        packet = packet[4:]

        return data_header, packet

    def parse_detected_points(self, packet, tlv_length):
        if tlv_length == 0:
            return np.zeros((0, 4))
        data_vals = self.parse_float_tlv_data(packet, tlv_length)
        data_vals = np.reshape(data_vals, (-1, 4))
        return data_vals
    
    def parse_detected_points_side_info(self, packet, tlv_length):
        if tlv_length == 0:
            return np.zeros((0, 4))
        data_vals = self.parse_int16_tlv_data(packet, tlv_length)
        scaled_data_vals = list()
        for val in data_vals:
            scaled_data_vals.append(float(val)/10)
        scaled_data_vals = np.reshape(scaled_data_vals, (-1, 2))
        return scaled_data_vals

    def read_parse_data(self):

        read_buffer = self.data_interface.read(self.data_interface.in_waiting)
        byte_vec = np.frombuffer(read_buffer, dtype='uint8')
        byte_count = len(byte_vec)
        if byte_count == 0:
            self.repeated_empty_reads += 1
            #if (self.repeated_empty_reads % MAX_REPEATED_EMPTY_READS) == 0:
            #    raise NoDataException(self.repeated_empty_reads)
        else:
            self.repeated_empty_reads = 0

        if (self.byte_buffer_len + byte_count) < self.byte_buffer.shape[0]:
            self.byte_buffer[self.byte_buffer_len:self.byte_buffer_len + byte_count] = byte_vec[:byte_count]
            self.byte_buffer_len = self.byte_buffer_len + byte_count

        packet = self.get_latest_packet()

        packet_header = None
        packet_data = None
        packet_format_error = False
        if packet is None:
            #print("Packet is None!")
            pass
        else:

            packet_header, packet = self.parse_header(packet)

            packet_data = dict()

            for tlv_idx in range(packet_header['num_TLVs']):
                tlv_type = struct.unpack('i', packet[0:4])[0]
                packet = packet[4:]
                tlv_length = struct.unpack('i', packet[0:4])[0]
                packet = packet[4:]

                if tlv_type == MMWDEMO_OUTPUT_MSG_TYPES['AZIMUT_ELEVATION_STATIC_HEAT_MAP']:
                    packet_data['AZIMUT_ELEVATION_STATIC_HEAT_MAP'] = self.parse_azimuth_elevation_range(packet,
                                                                                                         tlv_length,
                                                                                                         self.ti_config_params)
                    packet = packet[tlv_length:]
                elif tlv_type == MMWDEMO_OUTPUT_MSG_TYPES['RANGE_DOPPLER_HEAT_MAP']:
                    packet_data['RANGE_DOPPLER_HEAT_MAP'] = self.parse_range_doppler(packet, tlv_length,
                                                                                     self.ti_config_params)
                    current_sum = np.sum(packet_data['RANGE_DOPPLER_HEAT_MAP'])
                    if self.previous_doppler_sum is not None:
                        diff = abs(current_sum - self.previous_doppler_sum)
                        diff /= self.previous_doppler_sum
                        if diff < 0.1:
                            self.previous_doppler_sum = current_sum
                    else:
                        self.previous_doppler_sum = current_sum
                    packet = packet[tlv_length:]
                elif tlv_type == MMWDEMO_OUTPUT_MSG_TYPES['RANGE_PROFILE']:
                    packet_data['RANGE_PROFILE'] = self.parse_range_sum(packet, tlv_length, self.ti_config_params)
                    packet = packet[tlv_length:]
                elif tlv_type == MMWDEMO_OUTPUT_MSG_TYPES['NOISE_PROFILE']:
                    # We are re-purposing the NOISE_PROFILE for our Short-time FFT
                    packet_data['STFT'] = self.parse_range_profile(packet, tlv_length, self.ti_config_params)
                    packet = packet[tlv_length:]
                elif tlv_type == MMWDEMO_OUTPUT_MSG_TYPES['DETECTED_POINTS']:
                    packet_data['DETECTED_POINTS'] = self.parse_detected_points(packet, tlv_length)
                    packet = packet[tlv_length:]
                elif tlv_type == MMWDEMO_OUTPUT_MSG_TYPES['DETECTED_POINTS_SIDE_INFO']:
                    packet_data['DETECTED_POINTS_SIDE_INFO'] = self.parse_detected_points_side_info(packet, tlv_length)
                    packet = packet[tlv_length:]
                elif 0 < tlv_type < MMWDEMO_OUTPUT_MSG_TYPES['MAX']:
                    print('Unrecognized tlv type %d' % tlv_type)
                    packet = packet[tlv_length:]
                else:
                    print('error parsing tlv_type, skipping packet')
                    packet_format_error = True
                    break
        if packet_format_error:
            packet_header = None
            packet_data = None

        return packet_header, packet_data
