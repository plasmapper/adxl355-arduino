# Acceleration Waveform Measurement Example

Pin connections for using SPI or I2C interface
|ADXL355 pin|Arduino pin (SPI)|Arduino pin (I2C)|
|---|---|---|
|CS/SCL|2|SCL|
|MOSI/SDA|MOSI|SDA|
|MISO/ASEL|MISO|GND|
|SCLK/Vssio|SCLK|GND|

1. ADXL355 is initialized.
2. Range is set to ±2 g.
3. Measurement frequency is set to 4000 Hz.
4. Measurement is enabled.
5. Serial is initialized at 115200 kbps.
6. 10 time points of X-, Y- and Z-axis accelerations are read from the FIFO and printed every 5 seconds.
