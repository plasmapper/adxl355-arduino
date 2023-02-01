#include <PL_ADXL355.h>

//==============================================================================

// Create an instance of ADXL355
// ADXL355 MOSI, MISO and SCLK pins should be connected to the correspondent
// Arduino pins and ADXL355 CS pin should be connected to pin 2
PL::ADXL355 adxl355(2);

// ADXL355 range: +/- 2 g
auto range = PL::ADXL355_Range::range2g;

//==============================================================================

void setup() {
  // Initialize ADXL355
  adxl355.begin();
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
