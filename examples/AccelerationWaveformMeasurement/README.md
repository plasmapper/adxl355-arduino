# Acceleration Waveform Measurement Example

ADXL355 MOSI, MISO and SCLK pins should be connected to the correspondent Arduino pins and ADXL355 CS pin should be connected to pin 2.

1. ADXL355 is initialized.
2. Range is set to ±2 g.
3. Measurement frequency is set to 4000 Hz.
4. Measurement is enabled.
5. Serial is initialized at 115200 kbps.
6. 10 time points of X-, Y- and Z-axis accelerations are read from the FIFO and printed every 5 seconds.
