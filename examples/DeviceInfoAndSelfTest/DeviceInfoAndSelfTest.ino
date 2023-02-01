#include <PL_ADXL355.h>

//==============================================================================

// Create an instance of ADXL355
// ADXL355 MOSI, MISO and SCLK pins should be connected to the correspondent
// Arduino pins and ADXL355 CS pin should be connected to pin 2
PL::ADXL355 adxl355(2);

//==============================================================================

void setup() {
  // Initialize ADXL355
  adxl355.begin();
  
  // Initialize Serial at 115200 kbps
  Serial.begin(115200);
  // Wait for Serial ready state
  while(!Serial);

  // Reset the ADXL355
  adxl355.reset();

  // Read and print the device info
  auto deviceInfo = adxl355.getDeviceInfo();
  Serial.print("Vendor ID (should be 0xAD): 0x");
  Serial.println(deviceInfo.vendorId, HEX);
  Serial.print("Family ID (should be 0x1D): 0x");
  Serial.println(deviceInfo.familyId, HEX);
  Serial.print("Device ID (should be 0xED): 0x");
  Serial.println(deviceInfo.deviceId, HEX);
  Serial.print("Revision ID: 0x");
  Serial.println(deviceInfo.revisionId, HEX);

  // Execute the self-test and print the results
  Serial.println("");
  Serial.print ("Self test (should be 0.1...0.6 g, 0.1...0.6 g, 0.5...3.0 g): ");
  auto accelerations = adxl355.selfTest();
  Serial.print (accelerations.x);
  Serial.print (", ");
  Serial.print (accelerations.y);
  Serial.print (", ");
  Serial.println (accelerations.z);
}

//==============================================================================

void loop() {
}
