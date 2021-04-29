import pyaudio
import wave
import time

filename = 'youGotmail.wav'

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