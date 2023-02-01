# Acceleration Measurement Example

ADXL355 MOSI, MISO and SCLK pins should be connected to the correspondent Arduino pins and ADXL355 CS pin should be connected to pin 2.

1. ADXL355 is initialized.
2. Range is set to ±2 g.
3. Measurement is enabled.
4. Serial is initialized at 115200 kbps.
5. X-, Y- and Z-axis accelerations are measured and printed every second.
