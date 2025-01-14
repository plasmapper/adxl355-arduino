# Activity Detection Example

Pin connections for using SPI or I2C interface
|ADXL355 pin|Arduino pin (SPI)|Arduino pin (I2C)|
|---|---|---|
|CS/SCL|2|SCL|
|MOSI/SDA|MOSI|SDA|
|MISO/ASEL|MISO|GND|
|SCLK/Vssio|SCLK|GND|

1. ADXL355 is initialized.
2. Range is set to ±2 g.
3. Activity detection is enabled for X and Y axes with a threshold of 1.5 g and count 5.
4. Measurement is enabled.
5. Serial is initialized at 115200 kbps.
6. Activity detection is checked every second and the results are printed.
