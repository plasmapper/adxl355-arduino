#include "PL_ADXL355.h"

//==============================================================================

namespace PL {
  
//==============================================================================

const uint8_t ADXL355_REG_DEVID_AD = 0x00;
const uint8_t ADXL355_REG_DEVID_MST = 0x01;
const uint8_t ADXL355_REG_PARTID = 0x02;
const uint8_t ADXL355_REG_REVID = 0x03;
const uint8_t ADXL355_REG_STATUS = 0x04;
const uint8_t ADXL355_REG_FIFO_ENTRIES = 0x05;
const uint8_t ADXL355_REG_TEMP2 = 0x06;
const uint8_t ADXL355_REG_TEMP1 = 0x07;
const uint8_t ADXL355_REG_XDATA3 = 0x08;
const uint8_t ADXL355_REG_XDATA2 = 0x09;
const uint8_t ADXL355_REG_XDATA1 = 0x0A;
const uint8_t ADXL355_REG_YDATA3 = 0x0B;
const uint8_t ADXL355_REG_YDATA2 = 0x0C;
const uint8_t ADXL355_REG_YDATA1 = 0x0D;
const uint8_t ADXL355_REG_ZDATA3 = 0x0E;
const uint8_t ADXL355_REG_ZDATA2 = 0x0F;
const uint8_t ADXL355_REG_ZDATA1 = 0x10;
const uint8_t ADXL355_REG_FIFO_DATA = 0x11;
const uint8_t ADXL355_REG_OFFSET_X_H = 0x1E;
const uint8_t ADXL355_REG_OFFSET_X_L = 0x1F;
const uint8_t ADXL355_REG_OFFSET_Y_H = 0x20;
const uint8_t ADXL355_REG_OFFSET_Y_L = 0x21;
const uint8_t ADXL355_REG_OFFSET_Z_H = 0x22;
const uint8_t ADXL355_REG_OFFSET_Z_L = 0x23;
const uint8_t ADXL355_REG_ACT_EN = 0x24;
const uint8_t ADXL355_REG_ACT_THRESH_H = 0x25;
const uint8_t ADXL355_REG_ACT_THRESH_L = 0x26;
const uint8_t ADXL355_REG_ACT_COUNT = 0x27;
const uint8_t ADXL355_REG_FILTER = 0x28;
const uint8_t ADXL355_REG_FIFO_SAMPLES = 0x29;
const uint8_t ADXL355_REG_INT_MAP = 0x2A;
const uint8_t ADXL355_REG_SYNC = 0x2B;
const uint8_t ADXL355_REG_RANGE = 0x2C;
const uint8_t ADXL355_REG_POWER_CTL = 0x2D;
const uint8_t ADXL355_REG_SELF_TEST = 0x2E;
const uint8_t ADXL355_REG_RESET = 0x2F;

const uint8_t ADXL355_REG_STATUS_DATA_RDY = 0x01;
const uint8_t ADXL355_REG_STATUS_FIFO_FULL = 0x02;
const uint8_t ADXL355_REG_STATUS_FIFO_OVR = 0x04;
const uint8_t ADXL355_REG_STATUS_ACTIVITY = 0x08;
const uint8_t ADXL355_REG_STATUS_NVM_BUSY = 0x10;

const uint8_t ADXL355_REG_FILTER_HPF_SHIFT = 4;
const uint8_t ADXL355_REG_FILTER_HPF_MASK = 0xF0;
const uint8_t ADXL355_REG_FILTER_ODR_MASK = 0x0F;

const uint8_t ADXL355_REG_SYNC_SYNC_MAP = 0x03;
const uint8_t ADXL355_REG_SYNC_EXT_CLK = 0x04;

const uint8_t ADXL355_REG_RANGE_RANGE_MASK = 0x03;
const uint8_t ADXL355_REG_RANGE_RANGE_PM2G = 0x01;
const uint8_t ADXL355_REG_RANGE_RANGE_PM4G = 0x02;
const uint8_t ADXL355_REG_RANGE_RANGE_PM8G = 0x03;
const uint8_t ADXL355_REG_RANGE_INT_POL_SHIFT = 6;
const uint8_t ADXL355_REG_RANGE_INT_POL_MASK = 0x40;
const uint8_t ADXL355_REG_RANGE_I2C_HS_SHIFT = 7;
const uint8_t ADXL355_REG_RANGE_I2C_HS_MASK = 0x80;

const uint8_t ADXL355_REG_POWER_CTL_STANDBY = 0x01;
const uint8_t ADXL355_REG_POWER_CTL_TEMP_OFF = 0x02;
const uint8_t ADXL355_REG_POWER_CTL_DRDY_OFF = 0x04;

const uint8_t ADXL355_REG_SELF_TEST_ST1 = 0x01;
const uint8_t ADXL355_REG_SELF_TEST_ST2 = 0x02;

const uint8_t ADXL355_REG_RESET_RESET_CODE = 0x52;

//==============================================================================

ADXL355_Status operator|(ADXL355_Status status1, ADXL355_Status status2) {
  return (ADXL355_Status)((uint8_t)status1 | (uint8_t)status2);
}

//==============================================================================

ADXL355_Status operator&(ADXL355_Status status1, ADXL355_Status status2) {
  return (ADXL355_Status)((uint8_t)status1 & (uint8_t)status2);
}

//==============================================================================

ADXL355_Axes operator|(ADXL355_Axes axis1, ADXL355_Axes axis2) {
  return (ADXL355_Axes)((uint8_t)axis1 | (uint8_t)axis2);
}

//==============================================================================

ADXL355_Axes operator&(ADXL355_Axes axis1, ADXL355_Axes axis2) {
  return (ADXL355_Axes)((uint8_t)axis1 & (uint8_t)axis2);
}

//==============================================================================

ADXL355_Interrupts operator|(ADXL355_Interrupts axis1, ADXL355_Interrupts axis2) {
  return (ADXL355_Interrupts)((uint8_t)axis1 | (uint8_t)axis2);
}

//==============================================================================

ADXL355_Interrupts operator&(ADXL355_Interrupts axis1, ADXL355_Interrupts axis2) {
  return (ADXL355_Interrupts)((uint8_t)axis1 & (uint8_t)axis2);
}

//==============================================================================

ADXL355_RawAccelerations::ADXL355_RawAccelerations() : x(0), y(0), z(0) {}

//==============================================================================

ADXL355_RawAccelerations::ADXL355_RawAccelerations(int32_t x, int32_t y, int32_t z) : x(x), y(y), z(z) {}

//==============================================================================

ADXL355_Accelerations::ADXL355_Accelerations() : x(0), y(0), z(0) {}

//==============================================================================

ADXL355_Accelerations::ADXL355_Accelerations(float x, float y, float z) : x(x), y(y), z(z) {}

//==============================================================================

ADXL355::ADXL355 (uint8_t csPin, uint32_t frequency) : spiSettings (frequency, MSBFIRST, SPI_MODE0), csPin (csPin) {}

//==============================================================================

void ADXL355::begin() {
  SPI.begin();
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);
}

//==============================================================================

ADXL355_DeviceInfo ADXL355::getDeviceInfo() {
  uint8_t data[4];
  read (ADXL355_REG_DEVID_AD, &data, sizeof(data));
  ADXL355_DeviceInfo deviceInfo;
  deviceInfo.vendorId = data[0];
  deviceInfo.familyId = data[1];
  deviceInfo.deviceId = data[2];
  deviceInfo.revisionId = data[3];
  return deviceInfo;
}

//==============================================================================

ADXL355_Status ADXL355::getStatus() {
  return (ADXL355_Status)read(ADXL355_REG_STATUS);
}

//==============================================================================

uint8_t ADXL355::getNumberOfFifoSamples() {
  return read(ADXL355_REG_FIFO_ENTRIES);
}

//==============================================================================

uint16_t ADXL355::getRawTemperature() {
  uint8_t data[2];
  read(ADXL355_REG_TEMP2, &data, sizeof(data));
  uint16_t rawTemperature = 0;
  ((uint8_t*)&rawTemperature)[0] = data[1];
  ((uint8_t*)&rawTemperature)[1] = data[0];
  return rawTemperature;
}

//==============================================================================

float ADXL355::getTemperature() {
  return (((int16_t)getRawTemperature()) - ((int16_t)temperatureInterceptLsb)) / temperatureSlope + temperatureInterceptDegC;
}

//==============================================================================

ADXL355_RawAccelerations ADXL355::getRawAccelerations() {
  uint8_t data[9];
  ADXL355_RawAccelerations rawAccelerations(0, 0, 0);
  read(ADXL355_REG_XDATA3, &data, sizeof(data));
  
  ((uint8_t*)&rawAccelerations.x)[1] = data[2];
  ((uint8_t*)&rawAccelerations.x)[2] = data[1];
  ((uint8_t*)&rawAccelerations.x)[3] = data[0];
  
  ((uint8_t*)&rawAccelerations.y)[1] = data[5];
  ((uint8_t*)&rawAccelerations.y)[2] = data[4];
  ((uint8_t*)&rawAccelerations.y)[3] = data[3];
  
  ((uint8_t*)&rawAccelerations.z)[1] = data[8];
  ((uint8_t*)&rawAccelerations.z)[2] = data[7];
  ((uint8_t*)&rawAccelerations.z)[3] = data[6];
  
  rawAccelerations.x /= 4096;
  rawAccelerations.y /= 4096;
  rawAccelerations.z /= 4096;
  
  return rawAccelerations;
}

//==============================================================================

ADXL355_Accelerations ADXL355::getAccelerations() {
  ADXL355_RawAccelerations rawAccelerations = getRawAccelerations();
  float scaleFactor = getAccelerationScaleFactor();
  return ADXL355_Accelerations(rawAccelerations.x * scaleFactor, rawAccelerations.y * scaleFactor, rawAccelerations.z * scaleFactor);
}

//==============================================================================

void ADXL355::clearFifo() {
  uint8_t data[3];
  while(read(ADXL355_REG_FIFO_ENTRIES))
    read(ADXL355_REG_FIFO_DATA, &data, sizeof(data));    
}

//==============================================================================

ADXL355_RawAccelerations ADXL355::getRawAccelerationsFromFifo() {
  uint8_t data[3];
  ADXL355_RawAccelerations rawAccelerations(0, 0, 0);
  
  do {
    read(ADXL355_REG_FIFO_DATA, &data, sizeof(data));
  } while (!(data[2] & 0x01));
  
  if (data[2] & 0x02)
    return rawAccelerations;
    
  ((uint8_t*)&rawAccelerations.x)[1] = data[2];
  ((uint8_t*)&rawAccelerations.x)[2] = data[1];
  ((uint8_t*)&rawAccelerations.x)[3] = data[0];
  
  while(read(ADXL355_REG_FIFO_ENTRIES) < 2);
  
  read(ADXL355_REG_FIFO_DATA, &data, sizeof(data));
  ((uint8_t*)&rawAccelerations.y)[1] = data[2];
  ((uint8_t*)&rawAccelerations.y)[2] = data[1];
  ((uint8_t*)&rawAccelerations.y)[3] = data[0];
  
  read(ADXL355_REG_FIFO_DATA, &data, sizeof(data));
  ((uint8_t*)&rawAccelerations.z)[1] = data[2];
  ((uint8_t*)&rawAccelerations.z)[2] = data[1];
  ((uint8_t*)&rawAccelerations.z)[3] = data[0];
  
  rawAccelerations.x /= 4096;
  rawAccelerations.y /= 4096;
  rawAccelerations.z /= 4096;
  
  return rawAccelerations;
}

//==============================================================================

ADXL355_Accelerations ADXL355::getAccelerationsFromFifo() {
  ADXL355_RawAccelerations rawAccelerations = getRawAccelerationsFromFifo();
  float scaleFactor = getAccelerationScaleFactor();
  return ADXL355_Accelerations(rawAccelerations.x * scaleFactor, rawAccelerations.y * scaleFactor, rawAccelerations.z * scaleFactor);
}

//==============================================================================

ADXL355_RawAccelerations ADXL355::getRawOffsets() {
  uint8_t data[6];
  ADXL355_RawAccelerations rawOffsets(0, 0, 0);
  read(ADXL355_REG_OFFSET_X_H, &data, sizeof(data));
  ((uint8_t*)&rawOffsets.x)[2] = data[1];
  ((uint8_t*)&rawOffsets.x)[3] = data[0];
  ((uint8_t*)&rawOffsets.y)[2] = data[3];
  ((uint8_t*)&rawOffsets.y)[3] = data[2];
  ((uint8_t*)&rawOffsets.z)[2] = data[5];
  ((uint8_t*)&rawOffsets.z)[3] = data[4];
  rawOffsets.x /= 4096;
  rawOffsets.y /= 4096;
  rawOffsets.z /= 4096;
  return rawOffsets;
}

//==============================================================================

ADXL355_Accelerations ADXL355::getOffsets() {
  ADXL355_RawAccelerations rawOffsets = getRawOffsets();
  float scaleFactor = getAccelerationScaleFactor();
  return ADXL355_Accelerations(rawOffsets.x * scaleFactor, rawOffsets.y * scaleFactor, rawOffsets.z * scaleFactor);
}

//==============================================================================

void ADXL355::setRawOffsets(ADXL355_RawAccelerations rawOffsets) {
  uint8_t data[6];
  rawOffsets.x *= 4096;
  rawOffsets.y *= 4096;
  rawOffsets.z *= 4096;
  data[0] = ((uint8_t*)&rawOffsets.x)[3];
  data[1] = ((uint8_t*)&rawOffsets.x)[2];
  data[2] = ((uint8_t*)&rawOffsets.y)[3];
  data[3] = ((uint8_t*)&rawOffsets.y)[2];
  data[4] = ((uint8_t*)&rawOffsets.z)[3];
  data[5] = ((uint8_t*)&rawOffsets.z)[2];
  write(ADXL355_REG_OFFSET_X_H, &data, sizeof(data));
}

//==============================================================================

void ADXL355::setOffsets(ADXL355_Accelerations offsets) {
  float scaleFactor = getAccelerationScaleFactor();
  setRawOffsets (ADXL355_RawAccelerations(offsets.x / scaleFactor, offsets.y / scaleFactor, offsets.z / scaleFactor));
}

//==============================================================================

ADXL355_Axes ADXL355::getActivityDetectionAxes() {
  return (ADXL355_Axes)read(ADXL355_REG_ACT_EN);
}

//==============================================================================

void ADXL355::setActivityDetectionAxes(ADXL355_Axes axes) {
  write(ADXL355_REG_ACT_EN, (uint8_t)axes);
}

//==============================================================================

uint32_t ADXL355::getRawActivityDetectionThreshold() {
  uint8_t data[2];
  uint32_t rawThreshold = 0;
  read(ADXL355_REG_ACT_THRESH_H, &data, sizeof(data));
  ((uint8_t*)&rawThreshold)[0] = data[1];
  ((uint8_t*)&rawThreshold)[1] = data[0];
  rawThreshold <<= 3;
  return rawThreshold; 
}

//==============================================================================

float ADXL355::getActivityDetectionThreshold() {
  return getRawActivityDetectionThreshold() * getAccelerationScaleFactor();
}

//==============================================================================

void ADXL355::setRawActivityDetectionThreshold(uint32_t rawThreshold) {
  uint8_t data[2];
  rawThreshold >>= 3;
  data[0] = ((uint8_t*)&rawThreshold)[1];
  data[1] = ((uint8_t*)&rawThreshold)[0];
  write(ADXL355_REG_ACT_THRESH_H, &data, sizeof(data));
}

//==============================================================================

void ADXL355::setActivityDetectionThreshold(float threshold) {
  setRawActivityDetectionThreshold(threshold / getAccelerationScaleFactor());
}

//==============================================================================

uint8_t ADXL355::getActivityDetectionCount() {
  return read(ADXL355_REG_ACT_COUNT);
}

//==============================================================================

void ADXL355::setActivityDetectionCount(uint8_t count) {
  write(ADXL355_REG_ACT_COUNT, count);
}

//==============================================================================

ADXL355_HpfFrequency ADXL355::getHpfFrequency() {
  return (ADXL355_HpfFrequency)((read(ADXL355_REG_FILTER) & ADXL355_REG_FILTER_HPF_MASK) >> ADXL355_REG_FILTER_HPF_SHIFT);
}

//==============================================================================

void ADXL355::setHpfFrequency(ADXL355_HpfFrequency frequency) {
  uint8_t filterRegister = read(ADXL355_REG_FILTER) & ~ADXL355_REG_FILTER_HPF_MASK;
  write(ADXL355_REG_FILTER, filterRegister | ((uint8_t)frequency << ADXL355_REG_FILTER_HPF_SHIFT));
}

//==============================================================================

ADXL355_OutputDataRate ADXL355::getOutputDataRate() {
  return (ADXL355_OutputDataRate)(read(ADXL355_REG_FILTER) & ADXL355_REG_FILTER_ODR_MASK);
}

//==============================================================================

void ADXL355::setOutputDataRate(ADXL355_OutputDataRate outputDataRate) {
  uint8_t filterRegister = read(ADXL355_REG_FILTER) & ~ADXL355_REG_FILTER_ODR_MASK;
  write(ADXL355_REG_FILTER, filterRegister | (uint8_t)outputDataRate);
}

//==============================================================================

uint8_t ADXL355::getFifoWatermark() {
  return read(ADXL355_REG_FIFO_SAMPLES);
}

//==============================================================================

void ADXL355::setFifoWatermark(uint8_t watermark) {
  write(ADXL355_REG_FIFO_SAMPLES, watermark);
}

//==============================================================================

ADXL355_Interrupts ADXL355::getInterrupts() {
  return (ADXL355_Interrupts)read(ADXL355_REG_INT_MAP);
}

//==============================================================================

void ADXL355::setInterrupts(ADXL355_Interrupts interrupts) {
  write(ADXL355_REG_INT_MAP, (uint8_t)interrupts);
}

//==============================================================================

ADXL355_Synchronization ADXL355::getSynchronization() {
  return (ADXL355_Synchronization)(read(ADXL355_REG_SYNC) & ADXL355_REG_SYNC_SYNC_MAP);
}

//==============================================================================

void ADXL355::setSynchronization(ADXL355_Synchronization synchronization) {
  uint8_t syncRegister = read(ADXL355_REG_FILTER) & ~ADXL355_REG_SYNC_SYNC_MAP;
  write(ADXL355_REG_SYNC, syncRegister | (uint8_t)synchronization);
}

//==============================================================================

void ADXL355::enableExternalClock() {
  write(ADXL355_REG_SYNC, read(ADXL355_REG_SYNC) & ~ADXL355_REG_SYNC_EXT_CLK);
}

//==============================================================================

void ADXL355::disableExternalClock() {
  write(ADXL355_REG_SYNC, read(ADXL355_REG_SYNC) | ADXL355_REG_SYNC_EXT_CLK);
}

//==============================================================================

bool ADXL355::isExternalClockEnabled() {
  return !(read(ADXL355_REG_SYNC) & ADXL355_REG_SYNC_EXT_CLK);
}
  
//==============================================================================

ADXL355_Range ADXL355::getRange() {
  return (ADXL355_Range)(read(ADXL355_REG_RANGE) & ADXL355_REG_RANGE_RANGE_MASK);
}

//==============================================================================

void ADXL355::setRange(ADXL355_Range range) {
  uint8_t rangeRegister = read(ADXL355_REG_RANGE) & ~ADXL355_REG_RANGE_RANGE_MASK;
  write(ADXL355_REG_RANGE, rangeRegister | (uint8_t)range);
}

//==============================================================================

float ADXL355::getAccelerationScaleFactor() {
  switch (getRange()) {
    case ADXL355_Range::range2g:
      return accelerationScaleFactorRange2G;
    case ADXL355_Range::range4g:
      return accelerationScaleFactorRange4G;
    case ADXL355_Range::range8g:
      return accelerationScaleFactorRange8G;      
  }

  return 1;
}

//==============================================================================

ADXL355_InterruptPolarity ADXL355::getInterruptPolarity() {
  return (ADXL355_InterruptPolarity)((read(ADXL355_REG_RANGE) & ADXL355_REG_RANGE_INT_POL_MASK) >> ADXL355_REG_RANGE_INT_POL_SHIFT);
}

//==============================================================================

void ADXL355::setInterruptPolarity(ADXL355_InterruptPolarity polarity) {
  uint8_t rangeRegister = read(ADXL355_REG_RANGE) & ~ADXL355_REG_RANGE_INT_POL_MASK;
  write(ADXL355_REG_RANGE, rangeRegister | ((uint8_t)polarity << ADXL355_REG_RANGE_INT_POL_SHIFT));
}

//==============================================================================

ADXL355_I2CSpeed ADXL355::getI2CSpeed() {
  return (ADXL355_I2CSpeed)((read(ADXL355_REG_RANGE) & ADXL355_REG_RANGE_I2C_HS_MASK) >> ADXL355_REG_RANGE_I2C_HS_SHIFT);
}

//==============================================================================

void ADXL355::setI2CSpeed(ADXL355_I2CSpeed speed) {
  uint8_t rangeRegister = read(ADXL355_REG_RANGE) & ~ADXL355_REG_RANGE_I2C_HS_MASK;
  write(ADXL355_REG_RANGE, rangeRegister | ((uint8_t)speed << ADXL355_REG_RANGE_I2C_HS_SHIFT));
}

//==============================================================================

void ADXL355::enableMeasurement() {
  write(ADXL355_REG_POWER_CTL, read(ADXL355_REG_POWER_CTL) & ~ADXL355_REG_POWER_CTL_STANDBY);
}

//==============================================================================

void ADXL355::disableMeasurement() {
  write(ADXL355_REG_POWER_CTL, read(ADXL355_REG_POWER_CTL) | ADXL355_REG_POWER_CTL_STANDBY);
}

//==============================================================================

bool ADXL355::isMeasurementEnabled() {
  return !(read(ADXL355_REG_POWER_CTL) & ADXL355_REG_POWER_CTL_STANDBY);
}

//==============================================================================

void ADXL355::enableTemperature() {
  write(ADXL355_REG_POWER_CTL, read(ADXL355_REG_POWER_CTL) & ~ADXL355_REG_POWER_CTL_TEMP_OFF);
}

//==============================================================================

void ADXL355::disableTemperature() {
  write(ADXL355_REG_POWER_CTL, read(ADXL355_REG_POWER_CTL) | ADXL355_REG_POWER_CTL_TEMP_OFF);
}

//==============================================================================

bool ADXL355::isTemperatureEnabled() {
  return !(read(ADXL355_REG_POWER_CTL) & ADXL355_REG_POWER_CTL_TEMP_OFF);
}

//==============================================================================

void ADXL355::enableDataReady() {
  write(ADXL355_REG_POWER_CTL, read(ADXL355_REG_POWER_CTL) & ~ADXL355_REG_POWER_CTL_DRDY_OFF);
}

//==============================================================================

void ADXL355::disableDataReady() {
  write(ADXL355_REG_POWER_CTL, read(ADXL355_REG_POWER_CTL) | ADXL355_REG_POWER_CTL_DRDY_OFF);
}

//==============================================================================

bool ADXL355::isDataReadyEnabled() {
  return !(read(ADXL355_REG_POWER_CTL) & ADXL355_REG_POWER_CTL_DRDY_OFF);
}

//==============================================================================

ADXL355_Accelerations ADXL355::selfTest() {
  uint8_t measurementEnabled = isMeasurementEnabled();
  ADXL355_Accelerations accelNoForce, accelForce;

  setRange(ADXL355_Range::range8g);
  disableMeasurement();
  getAccelerations();
  write(ADXL355_REG_SELF_TEST, ADXL355_REG_SELF_TEST_ST1);
  delay(10);
  enableMeasurement();
  delay(10);
  accelNoForce = getAccelerations();
  write(ADXL355_REG_SELF_TEST, ADXL355_REG_SELF_TEST_ST2 | ADXL355_REG_SELF_TEST_ST1);
  delay(10);
  accelForce = getAccelerations();
  if (!measurementEnabled)
    disableMeasurement();
  write(ADXL355_REG_SELF_TEST, 0);
  return ADXL355_Accelerations(accelForce.x - accelNoForce.x, accelForce.y - accelNoForce.y, accelForce.z - accelNoForce.z);
}

//==============================================================================

void ADXL355::reset() {
  write(ADXL355_REG_RESET, ADXL355_REG_RESET_RESET_CODE);
}

//==============================================================================

uint8_t ADXL355::read(uint8_t address) {
  uint8_t data;
  read(address, &data, 1);
  return data;
}

//==============================================================================

void ADXL355::read(uint8_t address, void* dest, size_t numberOfRegisters) {
  if (!dest)
    return;

  SPI.beginTransaction(spiSettings);
  digitalWrite(csPin, LOW);
  SPI.transfer((address << 1) | 1);
  SPI.transfer(dest, numberOfRegisters);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

//==============================================================================

void ADXL355::write(uint8_t address, uint8_t value) {
  write(address, &value, 1);
}

//==============================================================================

void ADXL355::write(uint8_t address, void* src, size_t numberOfRegisters) {
  if (!src)
    return;

  SPI.beginTransaction(spiSettings);
  digitalWrite(csPin, LOW);
  SPI.transfer(address << 1);
  SPI.transfer(src, numberOfRegisters);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction(); 
}

//==============================================================================

}
