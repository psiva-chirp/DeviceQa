import time
import board
import busio
import adafruit_adxl34x
import adafruit_si7021
import adafruit_veml7700

i2c1 = busio.I2C(board.SCL, board.SDA)
i2c2 = busio.I2C(board.SCL, board.SDA)
i2c3 = busio.I2C(board.SCL, board.SDA)

# For ADXL343
accelerometer = adafruit_adxl34x.ADXL343(i2c1)
accelerometer.enable_motion_detection()

while True:
    try:
        sensor = adafruit_si7021.SI7021(i2c2)
        break
    except:
        continue
veml7700 = adafruit_veml7700.VEML7700(i2c3)

while True:
    print("******************************")
    print("\nAcceleration: %f %f %f" % accelerometer.acceleration)
    print("Motion detected: %s" % accelerometer.events["motion"])

    print("\nTemperature: %0.1f C" % sensor.temperature)
    print("Humidity: %0.1f %%" % sensor.relative_humidity)

    print("\nAmbient light:", veml7700.light)
    time.sleep(0.5)
