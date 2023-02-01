#include <PL_ADXL355.h>

//==============================================================================

// Create an instance of ADXL355
// ADXL355 MOSI, MISO and SCLK pins should be connected to the correspondent
// Arduino pins and ADXL355 CS pin should be connected to pin 2
PL::ADXL355 adxl355(2);

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
  // Initialize ADXL355
  adxl355.begin();
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
