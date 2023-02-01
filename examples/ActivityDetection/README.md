# Activity Detection Example

ADXL355 MOSI, MISO and SCLK pins should be connected to the correspondent Arduino pins and ADXL355 CS pin should be connected to pin 2.

1. ADXL355 is initialized.
2. Range is set to ±2 g.
3. Activity detection is enabled for X and Y axes with a threshold of 1.5 g and count 5.
4. Measurement is enabled.
5. Serial is initialized at 115200 kbps.
6. Activity detection is checked every second and the results are printed.
