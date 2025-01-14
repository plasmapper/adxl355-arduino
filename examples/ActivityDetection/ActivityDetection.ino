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
// Activity detection axes: X and Y
auto activityAxes = PL::ADXL355_Axes::x | PL::ADXL355_Axes::y;
// Activity detection threshold, g
float activityThreshold = 1.5;
// Activity detection count
uint8_t activityCount = 5;

//==============================================================================

void setup() {
  // Initialize ADXL355 (uncomment one of the following 2 lines to use either SPI or I2C)
  adxl355.beginSPI(spiCsPin);
  //adxl355.beginI2C(i2cAddress);

  // Set ADXL355 range to +/- 2 g
  adxl355.setRange(range);
  // Set activity detection axes
  adxl355.setActivityDetectionAxes(activityAxes);
  // Set activity detection threshold 
  adxl355.setActivityDetectionThreshold(activityThreshold);
  // Set activity detection count
  adxl355.setActivityDetectionCount(activityCount);
  // Enable ADXL355 measurement
  adxl355.enableMeasurement();
  
  // Initialize Serial at 115200 kbps
  Serial.begin(115200);
  // Wait for Serial ready state
  while(!Serial);
}

//==============================================================================

void loop() {
  // Check for an activity and print the result
  Serial.println((bool)(adxl355.getStatus() & PL::ADXL355_Status::activity) ? "Activity detected" : "No activity detected");

  delay (1000);
}
