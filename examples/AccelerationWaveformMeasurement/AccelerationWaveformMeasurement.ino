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
// ADXL355 output data rate (measurement frequency): 4000 Hz
auto outputDataRate = PL::ADXL355_OutputDataRate::odr1000;
// Measurement time step, ms
float timeStepMs = 1.0;
// Number of points to measure
const int numberOfPoints = 10;

//==============================================================================

void setup() {
  // Initialize ADXL355 (uncomment one of the following 2 lines to use either SPI or I2C)
  //adxl355.beginSPI(spiCsPin);
  adxl355.beginI2C(i2cAddress);
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

  // Wait for the required number of samples
  while (adxl355.getNumberOfFifoSamples() < numberOfPoints * 3);

  // Read the accelerations from the FIFO
  PL::ADXL355_Accelerations accelerations[numberOfPoints];
  for (int i = 0; i < numberOfPoints; i++)
    accelerations[i] = adxl355.getAccelerationsFromFifo();

  // Print the accelerations
  for (int i = 0; i < numberOfPoints; i++) {
    Serial.print("Time: ");
    Serial.print(i * timeStepMs);
    Serial.print(" ms, ");
    Serial.print("Accelerations: X: ");
    Serial.print(accelerations[i].x);
    Serial.print(" g, Y: ");
    Serial.print(accelerations[i].y);
    Serial.print(" g, Z: ");
    Serial.print(accelerations[i].z);
    Serial.println(" g");
  }
  Serial.println("");

  delay (5000);
}