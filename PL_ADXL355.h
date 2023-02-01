#ifndef PL_ADXL355_H
#define PL_ADXL355_H

#include <Arduino.h>
#include <SPI.h>

//==============================================================================

namespace PL {

//==============================================================================

/// @brief Device status
enum class ADXL355_Status : uint8_t {
  /// @brief a complete x-axis, y-axis, and z-axis measurement was made and results can be read
  dataReady = 0x01,
  /// @brief FIFO watermark is reached
  fifoFull = 0x02,
  /// @brief FIFO has overrun, and the oldest data is lost
  fifoOverrun = 0x04,
  /// @brief acceleration activity is detected
  activity = 0x08,
  /// @brief NVM controller is busy with a refresh, programming, or a built in self test
  nvmBusy = 0x10
};
ADXL355_Status operator|(ADXL355_Status status1, ADXL355_Status status2);
ADXL355_Status operator&(ADXL355_Status status1, ADXL355_Status status2);

//==============================================================================

/// @brief One or multiple axes
enum class ADXL355_Axes : uint8_t {
  /// @brief no axis
  none = 0,
  /// @brief X-axis
  x = 0x01,
  /// @brief Y-axis
  y = 0x02,
  /// @brief Z-axis
  z = 0x04
};
ADXL355_Axes operator|(ADXL355_Axes axes1, ADXL355_Axes axes2);
ADXL355_Axes operator&(ADXL355_Axes axes1, ADXL355_Axes axes2);

//==============================================================================

/// @brief High-pass filter frequency
enum class ADXL355_HpfFrequency : uint8_t {
  /// @brief high-pass filter disabled
  none = 0x00,
  /// @brief ODR*24.700e-4
  hpf24_7 = 0x01,
  /// @brief ODR*6.2084e-4
  hpf6_2084 = 0x02,
  /// @brief ODR*1.5545e-4
  hpf1_5545 = 0x03,
  /// @brief ODR*0.3862e-4
  hpf0_3862 = 0x04,
  /// @brief ODR*0.0954e-4
  hpf0_0954 = 0x05,
  /// @brief ODR*0.0238e-4
  hpf0_0238 = 0x06
};

//==============================================================================

/// @brief Output data rate
enum class ADXL355_OutputDataRate : uint8_t {
  /// @brief 4000 Hz (low-pass filter: 1000 Hz)
  odr4000 = 0x00,
  /// @brief 2000 Hz (low-pass filter: 500 Hz)
  odr2000 = 0x01,
  /// @brief 1000 Hz (low-pass filter: 250 Hz)
  odr1000 = 0x02,
  /// @brief 500 Hz (low-pass filter: 125 Hz)
  odr500 = 0x03,
  /// @brief 250 Hz (low-pass filter: 62.5 Hz)
  odr250 = 0x04,
  /// @brief 125 Hz (low-pass filter: 31.25 Hz)
  odr125 = 0x05,
  /// @brief 62.5 Hz (low-pass filter: 15.625 Hz)
  odr62_5 = 0x06,
  /// @brief 31.25 Hz (low-pass filter: 7.813 Hz)
  odr31_25 = 0x07,
  /// @brief 15.625 Hz (low-pass filter: 3.906 Hz)
  odr15_625 = 0x08,
  /// @brief 7.813 Hz (low-pass filter: 1.953 Hz)
  odr7_813 = 0x09,
  /// @brief 3.906 Hz (low-pass filter: 0.977 Hz)
  odr3_906 = 0x0A
};

//==============================================================================

/// @brief Interrupts
enum class ADXL355_Interrupts : uint8_t {
  /// @brief no interrupts
  none = 0x00,
  /// @brief DATA_RDY interrupt enable on INT1
  dataReadyInt1 = 0x01,
  /// @brief FIFO_FULL interrupt enable on INT1
  fifoFullInt1 = 0x02,
  /// @brief FIFO_OVR interrupt enable on INT1
  fifoOverrunInt1 = 0x04,
  /// @brief activity interrupt enable on INT1
  activityInt1 = 0x08,
  /// @brief DATA_RDY interrupt enable on INT2
  dataReadyInt2 = 0x10,
  /// @brief FIFO_FULL interrupt enable on INT2
  fifoFullInt2 = 0x20,
  /// @brief FIFO_OVR interrupt enable on INT2
  fifoOverrunInt2 = 0x40,
  /// @brief activity interrupt enable on INT2
  activityInt2 = 0x80
};
ADXL355_Interrupts operator|(ADXL355_Interrupts int1, ADXL355_Interrupts int2);
ADXL355_Interrupts operator&(ADXL355_Interrupts int1, ADXL355_Interrupts int2);

//==============================================================================

/// @brief Synchronization
enum class ADXL355_Synchronization : uint8_t {
  /// @brief internal
  internal = 0x00,
  /// @brief external
  external = 0x01,
  /// @brief external with interpolation filter
  externalWithInterpolation = 0x02
};

//==============================================================================

/// @brief Acceleration range
enum class ADXL355_Range : uint8_t {
  /// @brief ±2 g
  range2g = 0x01,
  /// @brief ±4 g
  range4g = 0x02,
  /// @brief ±8 g
  range8g = 0x03
};

//==============================================================================

/// @brief Interrupt polarity
enum class ADXL355_InterruptPolarity : uint8_t {
  /// @brief fast
  activeLow = 0x00,
  /// @brief high-speed
  activeHigh = 0x01
};

//==============================================================================

/// @brief Acceleration range
enum class ADXL355_I2CSpeed : uint8_t {
  /// @brief fast
  fast = 0x00,
  /// @brief high-speed
  highSpeed = 0x01
};

//==============================================================================

/// @brief Device information
struct ADXL355_DeviceInfo {
  /// @brief Vendor ID
  uint8_t vendorId;
  /// @brief Device family ID
  uint8_t familyId;
  /// @brief Device ID
  uint8_t deviceId;
  /// @brief Revision ID
  uint8_t revisionId;
};

//==============================================================================

/// @brief Raw accelerations
struct ADXL355_RawAccelerations {
  /// @brief Raw X-axis acceleration
  int32_t x;
  /// @brief Raw Y-axis acceleration
  int32_t y;
  /// @brief Raw Z-axis acceleration
  int32_t z;

  ADXL355_RawAccelerations();
  ADXL355_RawAccelerations(int32_t x, int32_t y, int32_t z);
};

//==============================================================================

/// @brief Accelerations in g
struct ADXL355_Accelerations {
  /// @brief X-axis acceleration, g
  float x;
  /// @brief Y-axis acceleration, g
  float y;
  /// @brief Z-axis acceleration, g
  float z;

  ADXL355_Accelerations();
  ADXL355_Accelerations(float x, float y, float z); 
};

//==============================================================================

/// @brief ADXL355 class
class ADXL355 {
public:
  /// @brief Default SPI frequency
  static constexpr uint32_t defaultSpiFrequency = 10000000;
  /// @brief Temperature intercept, LSB
  static constexpr uint16_t temperatureInterceptLsb = 1885;
  /// @brief Temperature intercept, °C
  static constexpr float temperatureInterceptDegC = 25;
  /// @brief Temperature slope, LSB/°C
  static constexpr float temperatureSlope = -9.05;
  /// @brief Acceleration scale factor for ±2 g range, g/LSB
  static constexpr float accelerationScaleFactorRange2G = 3.9e-6;
  /// @brief Acceleration scale factor for ±4 g range, g/LSB
  static constexpr float accelerationScaleFactorRange4G = 7.8e-6;
  /// @brief Acceleration scale factor for ±8 g range, g/LSB
  static constexpr float accelerationScaleFactorRange8G = 15.6e-6;
  /// @brief Maximum number of the FIFO samples
  static constexpr uint8_t maxNumberOfFifoSamples = 96;

  /// @brief Constructor
  /// @param csPin SPI chip select pin
  /// @param frequency SPI frequency, Hz
  ADXL355(uint8_t csPin, uint32_t frequency = defaultSpiFrequency);
  
  /// @brief Initializes the SPI bus and the CS pin
  void begin();

  /// @brief Gets the device information
  /// @return device information (should be vendorId: 0xAD, familyId: 0x1D, deviceId: 0xED)
  ADXL355_DeviceInfo getDeviceInfo();

  /// @brief Gets the device status
  /// @return device status
  ADXL355_Status getStatus();

  /// @brief Gets the number of valid data samples present in the FIFO buffer
  /// @return number of samples
  uint8_t getNumberOfFifoSamples();

  /// @brief Gets the raw temperature
  /// @return raw temperature (1885 LSB at 25°C, −9.05 LSB/°C)
  uint16_t getRawTemperature();

  /// @brief Gets the temperature
  /// @return temperature, °C
  float getTemperature();

  /// @brief Gets the raw X-, Y- and Z-axis accelerations
  /// @return raw accelerations
  ADXL355_RawAccelerations getRawAccelerations();

  /// @brief Gets the X-, Y- and Z-axis accelerations
  /// @return accelerations, g
  ADXL355_Accelerations getAccelerations();

  /// @brief Discards the valid 3-axis samples in the FIFO
  void clearFifo();

  /// @brief Gets the raw X-, Y- and Z-axis accelerations from the FIFO
  /// @return raw accelerations
  ADXL355_RawAccelerations getRawAccelerationsFromFifo();
  
  /// @brief Gets the X-, Y- and Z-axis accelerations from the FIFO
  /// @return accelerations, g
  ADXL355_Accelerations getAccelerationsFromFifo();

  /// @brief Gets the raw X-, Y- and Z-axis acceleration offsets
  /// @return raw acceleration offsets
  ADXL355_RawAccelerations getRawOffsets();

  /// @brief Gets the X-, Y- and Z-axis acceleration offsets
  /// @return acceleration offsets, g
  ADXL355_Accelerations getOffsets();

  /// @brief Sets the raw X-, Y- and Z-axis acceleration offsets
  /// @param rawOffsets raw acceleration offsets
  void setRawOffsets(ADXL355_RawAccelerations rawOffsets);

  /// @brief Sets the X-, Y- and Z-axis acceleration offsets
  /// @param offsets acceleration offsets, g
  void setOffsets(ADXL355_Accelerations offsets);

  /// @brief Gets the axes for which activity detection is enabled
  /// @return axes for which activity detection is enabled 
  ADXL355_Axes getActivityDetectionAxes();

  /// @brief Enables and disables activity detection
  /// @param axes axes for which activity detection should be enabled 
  void setActivityDetectionAxes(ADXL355_Axes axes);

  /// @brief Gets the raw activity detection threshold
  /// @return raw activity detection threshold
  uint32_t getRawActivityDetectionThreshold();

  /// @brief Gets the activity detection threshold
  /// @return activity detection threshold, g
  float getActivityDetectionThreshold();

  /// @brief Sets the raw activity detection threshold
  /// @param rawThreshold raw activity detection threshold (max: 524288)
  void setRawActivityDetectionThreshold(uint32_t rawThreshold);

  /// @brief Sets the activity detection threshold
  /// @param threshold activity detection threshold, g
  void setActivityDetectionThreshold(float threshold);

  /// @brief Gets the number of consecutive events above threshold required to detect activity
  /// @return activity count
  uint8_t getActivityDetectionCount();

  /// @brief Sets the number of consecutive events above threshold required to detect activity
  /// @param count activity count
  void setActivityDetectionCount(uint8_t count);

  /// @brief Gets the high-pass filter frequency
  /// @return high-pass filter frequency
  ADXL355_HpfFrequency getHpfFrequency();

  /// @brief Sets the high-pass filter frequency
  /// @param frequency high-pass filter frequency
  void setHpfFrequency(ADXL355_HpfFrequency frequency);

  /// @brief Gets the output data rate
  /// @return output data rate
  ADXL355_OutputDataRate getOutputDataRate();

  /// @brief Sets the output data rate
  /// @param outputDataRate output data rate
  void setOutputDataRate(ADXL355_OutputDataRate outputDataRate);

  /// @brief Gets the watermark number of samples stored in the FIFO that triggers a FIFO_FULL condition
  /// @return FIFO watermark
  uint8_t getFifoWatermark();

  /// @brief Sets the watermark number of samples stored in the FIFO that triggers a FIFO_FULL condition
  /// @param watermark output data rate
  void setFifoWatermark(uint8_t watermark);

  /// @brief Gets the interrupts
  /// @return interrupts
  ADXL355_Interrupts getInterrupts();
 
  /// @brief Sets the interrupts
  /// @param interrupts interrupts
  void setInterrupts(ADXL355_Interrupts interrupts);

  /// @brief Gets the synchronization mode
  /// @return synchronization mode
  ADXL355_Synchronization getSynchronization();
 
  /// @brief Sets the synchronization mode
  /// @param synchronization synchronization mode
  void setSynchronization(ADXL355_Synchronization synchronization);

  /// @brief Enables the external clock
  void enableExternalClock();

  /// @brief Disables the external clock
  void disableExternalClock();

  /// @brief Checksif the external clock is enabled
  /// @return true if the external clock is enabled
  bool isExternalClockEnabled();

  /// @brief Gets the acceleration range
  /// @return acceleration range
  ADXL355_Range getRange();
  
  /// @brief Gets the acceleration scale factor
  /// @return acceleration scale factor, g/LSB
  float getAccelerationScaleFactor();

  /// @brief Sets the acceleration range
  /// @param range acceleration range
  void setRange(ADXL355_Range range);

  /// @brief Gets the interrupt polarity
  /// @return interrupt polarity
  ADXL355_InterruptPolarity getInterruptPolarity();
 
  /// @brief Sets the interrupt polarity
  /// @param polarity interrupt polarity
  void setInterruptPolarity(ADXL355_InterruptPolarity polarity);

  /// @brief Gets the I2C speed
  /// @return I2C speed
  ADXL355_I2CSpeed getI2CSpeed();
 
  /// @brief Sets the I2C speed
  /// @param speed I2C speed
  void setI2CSpeed(ADXL355_I2CSpeed speed);

  /// @brief Switches mode to measurement mode
  void enableMeasurement();

  /// @brief Switches mode to standby mode
  void disableMeasurement();

  /// @brief Checks measurement mode
  /// @return true if measurement is enabled
  bool isMeasurementEnabled();
  
  /// @brief Enables the temperature processing
  void enableTemperature();

  /// @brief Disables the temperature processing
  void disableTemperature();

  /// @brief Checks if the temperature processing is enabled
  /// @return true if the temperature processing is enabled
  bool isTemperatureEnabled();

  /// @brief Enables the data-ready output
  void enableDataReady();

  /// @brief Disables the data-ready output
  void disableDataReady();

  /// @brief Checks if the data-ready output is enabled
  /// @return true if the data-ready output is enabled
  bool isDataReadyEnabled();
  
  /// @brief Performs the self-test of the device
  /// @return X-, Y- and Z-axis test accelerations (should be X: 0.1...0.6 g, Y: 0.1...0.6 g, Z: 0.5...3.0 g)
  ADXL355_Accelerations selfTest();

  /// @brief Resets the device
  void reset();

private:
  SPISettings spiSettings;
  uint8_t csPin;

  uint8_t read(uint8_t address);
  void read(uint8_t address, void* dest, size_t numberOfRegisters);
  void write(uint8_t address, uint8_t value);
  void write(uint8_t address, void* src, size_t numberOfRegisters);
};

//==============================================================================

}

#endif
