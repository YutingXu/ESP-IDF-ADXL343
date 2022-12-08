# ESP-IDF-ADXL343
ESP-IDF port of the Adafruit ADXL343 accelerometer driver breakout (https://www.adafruit.com/product/4097). However, it is written in C and is not based on Adafruit's Unified Sensor Library.

## About the ADXL343 ##

The ADXL343 is a digital accelerometer that supports both SPI and I2C mode, with adjustable data rata and 'range' (+/-2/4/8/16g).  The Adafruit_ADXL343 driver takes advantage of I2C mode to reduce the total pin count required to use the sensor.

More information on the ADXL345 can be found in the datasheet: http://www.analog.com/static/imported-files/data_sheets/ADXL343.pdf

## Device Compability:

Tested with ESPS3-DevKit using the I2C interface, SPI interface was not tested (as the breakout defaults to the I2C interface). It is expected that the library can be used with all ESP family devices.

## Further work & Contributions 

Any feedback is welcomed. More features are to be expected and can be added on-demand.
