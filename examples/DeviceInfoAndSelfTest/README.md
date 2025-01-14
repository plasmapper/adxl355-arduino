# Device Information and Self-Test Example

Pin connections for using SPI or I2C interface
|ADXL355 pin|Arduino pin (SPI)|Arduino pin (I2C)|
|---|---|---|
|CS/SCL|2|SCL|
|MOSI/SDA|MOSI|SDA|
|MISO/ASEL|MISO|GND|
|SCLK/Vssio|SCLK|GND|

1. ADXL355 is initialized.
2. Serial is initialized at 115200 kbps.
3. ADXL355 is reset.
4. Device information is read and printed.
5. Device self-test is executed and the results are printed.
