import pyaudio
import wave

def get_device_idx():
    p = pyaudio.PyAudio()
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')
    for i in range(0, numdevices):
        print(p.get_device_info_by_host_api_device_index(0, i).get('name'))
    p.terminate()

get_device_idx()