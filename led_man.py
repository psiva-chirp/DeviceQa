import spidev

class Color(object):
    def __init__(self, r, g, b):
        self.r = r
        self.g = g
        self.b = b

class LEDManager(object):
    def __init__(self):
        self.spi_bus = 0
        self.spi_dev = 0
        self.num_leds = 8
        self.set_brightness(15)
        self.pixels = [[0,0,0,0]] * self.num_leds
        self._open_spi()

    def _open_spi(self):
        self.spi = spidev.SpiDev()
        self.spi.open(self.spi_bus, self.spi_dev)

    #These 4 bytes have to be written at the start and
    #end of each data frame when writing to the LEDs
    def _start_end_frame(self):
        for x in range(4):
            self.spi.xfer2([0x00])

    #Write data to the LEDs
    def _write_leds(self):
        self._start_end_frame()
        for LED in self.pixels:
            r, g, b, brightness = LED
            self.spi.xfer2([brightness])
            self.spi.xfer2([b])
            self.spi.xfer2([g])
            self.spi.xfer2([r])

        self._start_end_frame()

    def _buffer_pixel(self, index, color):
        self.pixels[index] = [color.r, color.g, color.b, self.brightness]

    def set_brightness(self, brightness):
        if (brightness > 31) | (brightness < 0):
            brightness = 15
        self.brightness = brightness | 0xE0

    def set_pixel(self, index, color):
        self._buffer_pixel(index, color)
        self._write_leds()

    def off_pixels(self):
        for i in range(self.num_leds):
            self._buffer_pixel(i, Color(0,0,0))
        self._write_leds()

import time
ledman = LEDManager()
ledman.off_pixels()
for i in range(0, 8):
    ledman.set_pixel(i, Color(0, 0, 20))
    time.sleep(0.1)

for i in range(0,8):
    ledman.set_pixel(7-i, Color(20, 0, 0))
    time.sleep(0.1)

for i in range(0,8):
    ledman.set_pixel(i, Color(0,0,0))
    time.sleep(0.1)