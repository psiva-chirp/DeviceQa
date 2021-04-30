import pyaudio
import wave
import time

def get_device_idx(name='dmic-sv'):
    p = pyaudio.PyAudio()
    device_idx = None
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')
    for i in range(0, numdevices):
        if (p.get_device_info_by_host_api_device_index(0, i).get('name') == name):
            device_idx = i
            break
    p.terminate()
    return device_idx

filename_out = 'youGotmail2.wav'


chunk = 1024

wf = wave.open(filename_out, 'rb')

p = pyaudio.PyAudio()

print(pyaudio.paInt32)
print(p.get_format_from_width(pyaudio.paFloat32))
print(p.get_format_from_width(wf.getsampwidth()))
print(wf.getnchannels())
print(wf.getframerate())

channels = wf.getnchannels()
FORMAT = p.get_format_from_width(wf.getsampwidth())
sample_rate = wf.getframerate()

print('device')
print(get_device_idx(name='lp'))

stream = p.open(format = FORMAT, 
    channels = channels, 
    rate = sample_rate, 
    output = True,
    input = False,
    output_device_index=get_device_idx(name='default'))

streamin = p.open(format = pyaudio.paInt32, 
    channels = 1, 
    rate = 44100, 
    output = False,
    input = True,
    frames_per_buffer=chunk*3,
    input_device_index=get_device_idx(name='lp'))

frames = []
data = wf.readframes(chunk)

while data != b'':
    data_i = streamin.read(chunk)
    stream.write(data)
    #data_i = streamin.read(chunk)
    #frames.append(data_i)
    data = wf.readframes(chunk)
    print('a')
time.sleep(0.5)

stream.close()
p.terminate()

print(len(frames))


# save audio file
# open the file in 'write bytes' mode
wf = wave.open('playRecord.wav', "wb")
# set the channels
wf.setnchannels(channels)
# set the sample format
wf.setsampwidth(p.get_sample_size(FORMAT))
# set the sample rate
wf.setframerate(sample_rate)
# write the frames as bytes
wf.writeframes(b"".join(frames))
# close the file
wf.close()