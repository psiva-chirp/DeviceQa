import pyaudio
import wave

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

# the file name output you want to record into
filename = "recorded.wav"
# set the chunk size of 1024 samples
chunk = 1024
# sample format
FORMAT = pyaudio.paInt32
# mono, change to 2 if you want stereo
channels = 1
# 44100 samples per second
sample_rate = 44100
record_seconds = 30

device_idx = get_device_idx(name='dmic_sv')

# initialize PyAudio object
p = pyaudio.PyAudio()


# open stream object as input & output
stream = p.open(format=FORMAT,
                channels=channels,
                rate=sample_rate,
                input=True,
                output=False,
                frames_per_buffer=chunk,
                input_device_index=device_idx)
frames = []
print("Recording...")
for i in range(int(44100 / chunk * record_seconds)):
    data = stream.read(chunk)
    frames.append(data)
print("Finished recording.")
# stop and close stream
stream.stop_stream()
stream.close()
# terminate pyaudio object
p.terminate()
# save audio file
# open the file in 'write bytes' mode
wf = wave.open(filename, "wb")
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
