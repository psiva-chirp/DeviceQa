import time
import board
import neopixel

# On a Raspberry pi, use this instead, not all pins are supported
pixel_pin = board.D12
 
# The number of NeoPixels
num_pixels = 8
 
ORDER = neopixel.GRBW
 
pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=0.5, pixel_order=ORDER
)

for i in range(0,8):
    pixels[i] = (255, 0, 0, 0)
    time.sleep(0.1)
pixels.fill((0, 0, 255, 0))
time.sleep(1)
for i in range(0,8):
    pixels[7-i] = (0, 0, 0, 0)
    time.sleep(0.1)
