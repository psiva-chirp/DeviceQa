import pyaudio
import wave
import time

#arecord -D plughw:0 -c1 -r 48000 -f S32_LE | aplay -D plughw:0 -c1 -r48000 -f S32_LE

def get_device_idx(name):
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

filename = "recording.wav"
chunk = 1024
FORMAT = pyaudio.paFloat32 #pyaudio.paInt32
channels = 1
sample_rate = 44100
record_seconds = 10

device_idx = get_device_idx(name='lp')

p = pyaudio.PyAudio()

stream = p.open(format=FORMAT,
                channels=channels,
                rate=sample_rate,
                input=True,
                output=False,
                frames_per_buffer=chunk,
                input_device_index=device_idx)
frames = []
print("Recording...")
for i in range(int(sample_rate / chunk * record_seconds)):
    data = stream.read(chunk)
    frames.append(data)
print("Finished recording.")

stream.stop_stream()
stream.close()
p.terminate()

# save audio file
wf = wave.open(filename, "wb")
wf.setnchannels(channels)
wf.setsampwidth(p.get_sample_size(FORMAT))
wf.setframerate(sample_rate)
wf.writeframes(b"".join(frames))
wf.close()


chunk = 1024

wf = wave.open(filename, 'rb')
p = pyaudio.PyAudio()
stream = p.open(format = p.get_format_from_width(wf.getsampwidth()), channels = wf.getnchannels(), rate = wf.getframerate(), output = True)
data = wf.readframes(chunk)

while data != b'':
    stream.write(data)
    data = wf.readframes(chunk)
time.sleep(0.5)

stream.close()
p.terminate()