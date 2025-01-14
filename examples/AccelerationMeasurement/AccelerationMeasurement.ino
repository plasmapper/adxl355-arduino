#include <PL_ADXL355.h>

//==============================================================================

// Pin connections for using SPI or I2C interface
// ADXL355 pin    Arduino pin
//                SPI     I2C
// CS/SCL          2      SCL
// MOSI/SDA       MOSI    SDA
// MISO/ASEL      MISO    GND
// SCLK/Vssio     SCLK    GND

PL::ADXL355 adxl355;
uint8_t spiCsPin = 2;
uint8_t i2cAddress = 0x1D;

// ADXL355 range: +/- 2 g
auto range = PL::ADXL355_Range::range2g;

//==============================================================================

void setup() {
  // Initialize ADXL355 (uncomment one of the following 2 lines to use either SPI or I2C)
  adxl355.beginSPI(spiCsPin);
  //adxl355.beginI2C(i2cAddress);

  // Set ADXL355 range
  adxl355.setRange(range);
  // Enable ADXL355 measurement
  adxl355.enableMeasurement();
  
  // Initialize Serial at 115200 kbps
  Serial.begin(115200);
  // Wait for Serial ready state
  while(!Serial);
}

//==============================================================================

void loop() {
  // Read and print the accelerations
  auto accelerations = adxl355.getAccelerations();
  Serial.print("Accelerations: X: ");
  Serial.print(accelerations.x);
  Serial.print(" g, Y: ");
  Serial.print(accelerations.y);
  Serial.print(" g, Z: ");
  Serial.print(accelerations.z);
  Serial.println(" g");

  delay (1000);
}
