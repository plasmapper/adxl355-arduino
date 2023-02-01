#include <PL_ADXL355.h>

//==============================================================================

// Create an instance of ADXL355 with CS pin connected to pin 2
// ADXL355 MOSI, MISO and SCLK pins should be connected to the correspondent
// Arduino pins and ADXL355 CS pin should be connected to pin 2
PL::ADXL355 adxl355(2);

// ADXL355 range: +/- 2 g
auto range = PL::ADXL355_Range::range2g;
// ADXL355 output data rate (measurement frequency): 4000 Hz
auto outputDataRate = PL::ADXL355_OutputDataRate::odr4000;
// Measurement time step, ms
float timeStepMs = 1.0 / 4000.0 * 1000;
// Number of points to measure
int numberOfPoints = 10;

//==============================================================================

void setup() {
  // Initialize ADXL355
  adxl355.begin();
  // Set ADXL355 range
  adxl355.setRange(range);
  // Set ADXL355 output data rate
  adxl355.setOutputDataRate(outputDataRate);
  // Enable ADXL355 measurement
  adxl355.enableMeasurement();
  
  // Initialize Serial at 115200 kbps
  Serial.begin(115200);
  // Wait for Serial ready state
  while(!Serial);
}

//==============================================================================

void loop() {
  // Clear the FIFO
  adxl355.clearFifo();

  for (int i = 0; i < numberOfPoints; i++) {
    // Read and print the accelerations from the FIFO
    auto accelerations = adxl355.getAccelerationsFromFifo();
    Serial.print("Time: ");
    Serial.print(i * timeStepMs);
    Serial.print(" ms, ");
    Serial.print("Accelerations: X: ");
    Serial.print(accelerations.x);
    Serial.print(" g, Y: ");
    Serial.print(accelerations.y);
    Serial.print(" g, Z: ");
    Serial.print(accelerations.z);
    Serial.println(" g");
  }
  Serial.println("");

  delay (5000);
}
