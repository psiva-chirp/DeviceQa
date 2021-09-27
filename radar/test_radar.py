from time import sleep
from stream_radar import Stream6843AOP

with open('temp_radar_config.txt', 'r') as fid:
    config_str = fid.read()

streamer = Stream6843AOP(config_str, '/dev/ttyS2', '/dev/ttyS3')

while True:
    try:
        packet_header, packet_data = streamer.read_parse_data()
        if packet_header is None:
            continue
        print(packet_header)
        print(packet_data)
        sleep(0.001)

    except KeyboardInterrupt:
        break
print('stop sensor')
streamer.stop_sensor()
print('close ports')
streamer.close_ports()
print('stopping threads')