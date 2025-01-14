#include <ArduinoUnit.h>
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

//==============================================================================

void setup() {
  // Initialize ADXL355 (uncomment one of the following 2 lines to use either SPI or I2C)
  adxl355.beginSPI(spiCsPin);
  //adxl355.beginI2C(i2cAddress);

  randomSeed(analogRead(5));
  Serial.begin(115200);
  while(!Serial);
}

//==============================================================================

void loop() {
  Test::run();
}

//==============================================================================

test(deviceId) {
  auto deviceInfo = adxl355.getDeviceInfo();
  assertEqual(deviceInfo.vendorId, 0xAD);
  assertEqual(deviceInfo.familyId, 0x1D);
  assertEqual(deviceInfo.deviceId, 0xED);
}

//==============================================================================

test(getNumberOfFifoSamples) {
  adxl355.reset();
  adxl355.setOutputDataRate(PL::ADXL355_OutputDataRate::odr4000);
  adxl355.enableMeasurement();
  delay(50);
  assertEqual(adxl355.getNumberOfFifoSamples(), PL::ADXL355::maxNumberOfFifoSamples);
}

//==============================================================================

test(offsets) {
  float scaleFactor = adxl355.getAccelerationScaleFactor();
  
  PL::ADXL355_RawAccelerations rawOffsetsToSet(1000, 2000, -4000);
  adxl355.setRawOffsets(rawOffsetsToSet);
  
  auto rawOffsetsSet = adxl355.getRawOffsets();
  assertNear(rawOffsetsToSet.x, rawOffsetsSet.x, 16);
  assertNear(rawOffsetsToSet.y, rawOffsetsSet.y, 16);
  assertNear(rawOffsetsToSet.z, rawOffsetsSet.z, 16);
  auto offsetsSet = adxl355.getOffsets();
  assertNear(rawOffsetsToSet.x * scaleFactor, offsetsSet.x, 16 * scaleFactor);
  assertNear(rawOffsetsToSet.y * scaleFactor, offsetsSet.y, 16 * scaleFactor);
  assertNear(rawOffsetsToSet.z * scaleFactor, offsetsSet.z, 16 * scaleFactor);

  PL::ADXL355_Accelerations offsetsToSet(0.1, 0.2, -0.4);
  adxl355.setOffsets(offsetsToSet);
  
  rawOffsetsSet = adxl355.getRawOffsets();
  assertNear(offsetsToSet.x / scaleFactor, rawOffsetsSet.x, 16);
  assertNear(offsetsToSet.y / scaleFactor, rawOffsetsSet.y, 16);
  assertNear(offsetsToSet.z / scaleFactor, rawOffsetsSet.z, 16);
  offsetsSet = adxl355.getOffsets();
  assertNear(offsetsToSet.x, offsetsSet.x, 16 * scaleFactor);
  assertNear(offsetsToSet.y, offsetsSet.y, 16 * scaleFactor);
  assertNear(offsetsToSet.z, offsetsSet.z, 16 * scaleFactor);
}

//==============================================================================

test(activityDetectionAxes) {
  PL::ADXL355_Axes axes = PL::ADXL355_Axes::x | PL::ADXL355_Axes::z;
  adxl355.setActivityDetectionAxes(axes);
  assertEqual((uint8_t)axes, (uint8_t)adxl355.getActivityDetectionAxes());
}

//==============================================================================

test(activityThreshold) {
  float scaleFactor = adxl355.getAccelerationScaleFactor();
  
  uint32_t rawThresholdToSet = 1000;
  adxl355.setRawActivityDetectionThreshold(rawThresholdToSet);
  
  uint32_t rawThresholdSet = adxl355.getRawActivityDetectionThreshold();
  assertNear(rawThresholdToSet, rawThresholdSet, 8);
  float thresholdSet = adxl355.getActivityDetectionThreshold();
  assertNear(rawThresholdToSet * scaleFactor, thresholdSet, 8 * scaleFactor);

  float thresholdToSet = 0.1;
  adxl355.setActivityDetectionThreshold(thresholdToSet);
  
  rawThresholdSet = adxl355.getRawActivityDetectionThreshold();
  assertNear(thresholdToSet / scaleFactor, rawThresholdSet, 8);
  thresholdSet = adxl355.getActivityDetectionThreshold();
  assertNear(thresholdToSet, thresholdSet, 8 * scaleFactor);
}

//==============================================================================

test(activityCount) {
  uint8_t activityCount = 10;
  adxl355.setActivityDetectionCount(activityCount);
  assertEqual(activityCount, adxl355.getActivityDetectionCount());
}

//==============================================================================

test(hpfFrequencyAndOutputDataRate) {
  for (uint8_t freq = 0; freq <= (uint8_t)PL::ADXL355_HpfFrequency::hpf0_0238; freq++) {
    for (uint8_t odr = 0; odr <= (uint8_t)PL::ADXL355_OutputDataRate::odr3_906; odr++) {
      adxl355.setHpfFrequency((PL::ADXL355_HpfFrequency)freq);
      adxl355.setOutputDataRate((PL::ADXL355_OutputDataRate)odr);
      assertEqual(freq, (uint8_t)adxl355.getHpfFrequency());
      assertEqual(odr, (uint8_t)adxl355.getOutputDataRate());
    }
  }
}

//==============================================================================

test(fifoWatermark) {
  uint8_t watermark = 15;
  adxl355.setFifoWatermark(watermark);
  assertEqual(watermark, adxl355.getFifoWatermark());
}

//==============================================================================

test(interrupts) {
  PL::ADXL355_Interrupts interrupts = PL::ADXL355_Interrupts::dataReadyInt1 | PL::ADXL355_Interrupts::fifoFullInt2;
  adxl355.setInterrupts(interrupts);
  assertEqual((uint8_t)interrupts, (uint8_t)adxl355.getInterrupts());
}

//==============================================================================

test(synchronizationAndExternalClock) {
  for (uint8_t sync = 0; sync <= (uint8_t)PL::ADXL355_Synchronization::externalWithInterpolation; sync++) {
    for (uint8_t extClock = 0; extClock <= 1; extClock++) {
      adxl355.setSynchronization((PL::ADXL355_Synchronization)sync);
      if (extClock)
        adxl355.enableExternalClock();
      else
        adxl355.disableExternalClock();
      delay(1);
      assertEqual(sync, (uint8_t)adxl355.getSynchronization());
      assertEqual(extClock, (uint8_t)adxl355.isExternalClockEnabled());
    }
  }
}

//==============================================================================

test(rangeIntPolarityAndI2CSpeed) {
  for (uint8_t range = 0; range <= (uint8_t)PL::ADXL355_Range::range8g; range++) {
    for (uint8_t intPol = 0; intPol <= (uint8_t)PL::ADXL355_InterruptPolarity::activeHigh; intPol++) {
      for (uint8_t i2CSpeed = 0; i2CSpeed <= (uint8_t)PL::ADXL355_I2CSpeed::highSpeed; i2CSpeed++) {
        adxl355.setRange((PL::ADXL355_Range)range);
        adxl355.setInterruptPolarity((PL::ADXL355_InterruptPolarity)intPol);
        adxl355.setI2CSpeed((PL::ADXL355_I2CSpeed)i2CSpeed);
        assertEqual(range, (uint8_t)adxl355.getRange());
        switch ((PL::ADXL355_Range)range) {
          case PL::ADXL355_Range::range2g:
            assertEqual (PL::ADXL355::accelerationScaleFactorRange2G, adxl355.getAccelerationScaleFactor());
            break;
          case PL::ADXL355_Range::range4g:
            assertEqual (PL::ADXL355::accelerationScaleFactorRange4G, adxl355.getAccelerationScaleFactor());
            break;
          case PL::ADXL355_Range::range8g:
            assertEqual (PL::ADXL355::accelerationScaleFactorRange8G, adxl355.getAccelerationScaleFactor());
            break;
        }
        assertEqual(intPol, (uint8_t)adxl355.getInterruptPolarity());
        assertEqual(i2CSpeed, (uint8_t)adxl355.getI2CSpeed());
      }
    }
  }
}

//==============================================================================

test(power) {
  for (uint8_t measurement = 0; measurement <= 1; measurement++) {
    for (uint8_t temperature = 0; temperature <= 1; temperature++) {
      for (uint8_t dataReady = 0; dataReady <= 1; dataReady++) {
        if (measurement)
          adxl355.enableMeasurement();
        else
          adxl355.disableMeasurement();
        if (temperature)
          adxl355.enableTemperature();
        else
          adxl355.disableTemperature();
        if (dataReady)
          adxl355.enableDataReady();
        else
          adxl355.disableDataReady();

        assertEqual(measurement, (uint8_t)adxl355.isMeasurementEnabled());
        assertEqual(temperature, (uint8_t)adxl355.isTemperatureEnabled());
        assertEqual(dataReady, (uint8_t)adxl355.isDataReadyEnabled()); 
      }
    }
  }
}

//==============================================================================

test(reset) {
  adxl355.reset();
  adxl355.setOutputDataRate(PL::ADXL355_OutputDataRate::odr4000);
  adxl355.enableMeasurement();
  delay(50);
  assertEqual(adxl355.getNumberOfFifoSamples(), PL::ADXL355::maxNumberOfFifoSamples);
  adxl355.reset();
  assertEqual(adxl355.getNumberOfFifoSamples(), 0);
}
