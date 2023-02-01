Analog Devices ADXL355 Accelerometer Library for Arduino
========================================================

Usage
-----
Use :cpp:func:`PL::ADXL355::ADXL355` to create an instance of ADXL355 and
:cpp:func:`PL::ADXL355::begin` to initialize it.

Use :cpp:func:`PL::ADXL355::getRange` and :cpp:func:`PL::ADXL355::setRange`
to configure the measurement range and :cpp:func:`PL::ADXL355::getAccelerationScaleFactor`
to get the acceleration scale factor in g/LSB.

Use :cpp:func:`PL::ADXL355::getOutputDataRate` and :cpp:func:`PL::ADXL355::setOutputDataRate`
to configure the measurement frequency.

Use :cpp:func:`PL::ADXL355::enableMeasurement` to enable the acceleration measurement,
:cpp:func:`PL::ADXL355::disableMeasurement` to disable the acceleration measurement and
:cpp:func:`PL::ADXL355::isMeasurementEnabled` to check if the acceleration measurement is enabled.

Use :cpp:func:`PL::ADXL355::getAccelerations`/:cpp:func:`PL::ADXL355::getRawAccelerations`
to measure the instantaneous acceleration values.

Use :cpp:func:`PL::ADXL355::getNumberOfFifoSamples`, :cpp:func:`PL::ADXL355::clearFifo` and
:cpp:func:`PL::ADXL355::getAccelerationsFromFifo`/:cpp:func:`PL::ADXL355::getRawAccelerationsFromFifo`
to get the acceleration waveforms.

Use :cpp:func:`PL::ADXL355::getOffsets`/:cpp:func:`PL::ADXL355::getRawOffsets` and
:cpp:func:`PL::ADXL355::setOffsets`/:cpp:func:`PL::ADXL355::setRawOffsets`
to configure the acceleration offsets.

Use :cpp:func:`PL::ADXL355::getHpfFrequency` and :cpp:func:`PL::ADXL355::setHpfFrequency`
to configure the high-pass filter.

Use :cpp:func:`PL::ADXL355::getActivityDetectionAxes`, :cpp:func:`PL::ADXL355::setActivityDetectionAxes`,
:cpp:func:`PL::ADXL355::getActivityDetectionThreshold`/:cpp:func:`PL::ADXL355::getRawActivityDetectionThreshold`,
:cpp:func:`PL::ADXL355::setActivityDetectionThreshold`/:cpp:func:`PL::ADXL355::setRawActivityDetectionThreshold`,
:cpp:func:`PL::ADXL355::getActivityDetectionCount` and :cpp:func:`PL::ADXL355::setActivityDetectionCount`
to configure the acceleration activity detection.

Use :cpp:func:`PL::ADXL355::enableTemperature` to enable the temperature measurement,
:cpp:func:`PL::ADXL355::disableTemperature` to disable the temperature measurement and
:cpp:func:`PL::ADXL355::isTemperatureEnabled` to check if the temperature measurement is enabled.

Use :cpp:func:`PL::ADXL355::getTemperature`/:cpp:func:`PL::ADXL355::getRawTemperature`
to measure temperature.

Use :cpp:func:`PL::ADXL355::getInterrupts` and :cpp:func:`PL::ADXL355::setInterrupts`
to configure the interrupts.

Use :cpp:func:`PL::ADXL355::getStatus`
to get the device status.

Use :cpp:func:`PL::ADXL355::selfTest`
to perform the self-test of the device.

Use :cpp:func:`PL::ADXL355::getDeviceInfo`
to get the device information.

Use :cpp:func:`PL::ADXL355::reset`
to reset the device.

Other functions:
:cpp:func:`PL::ADXL355::getFifoWatermark`, :cpp:func:`PL::ADXL355::setFifoWatermark`,
:cpp:func:`PL::ADXL355::getSynchronization`, :cpp:func:`PL::ADXL355::setSynchronization`,
:cpp:func:`PL::ADXL355::enableExternalClock`, :cpp:func:`PL::ADXL355::disableExternalClock`,
:cpp:func:`PL::ADXL355::isExternalClockEnabled`
:cpp:func:`PL::ADXL355::getInterruptPolarity`, :cpp:func:`PL::ADXL355::setInterruptPolarity`,
:cpp:func:`PL::ADXL355::getI2CSpeed`, :cpp:func:`PL::ADXL355::setI2CSpeed`,
:cpp:func:`PL::ADXL355::enableDataReady`, :cpp:func:`PL::ADXL355::disableDataReady`,
:cpp:func:`PL::ADXL355::isDataReadyEnabled`.

Refer to the `ADXL355 <https://www.analog.com/en/products/adxl355.html>`_ datasheet for the details
on the device usage and configuration.

Examples
--------
| `Device information and self test <https://github.com/plasmapper/adxl355-arduino/tree/main/examples/DeviceInfoAndSelfTest>`_
| `Acceleration measurement <https://github.com/plasmapper/adxl355-arduino/tree/main/examples/AccelerationMeasurement>`_
| `Acceleration waveform measurement <https://github.com/plasmapper/adxl355-arduino/tree/main/examples/AccelerationWaveformMeasurement>`_
| `Activity detection <https://github.com/plasmapper/adxl355-arduino/tree/main/examples/ActivityDetection>`_

API reference
-------------

.. doxygenclass:: PL::ADXL355
  :members:
  :protected-members:

.. doxygenenum:: PL::ADXL355_Status

.. doxygenenum:: PL::ADXL355_Axes

.. doxygenenum:: PL::ADXL355_HpfFrequency

.. doxygenenum:: PL::ADXL355_OutputDataRate

.. doxygenenum:: PL::ADXL355_Interrupts

.. doxygenenum:: PL::ADXL355_Synchronization

.. doxygenenum:: PL::ADXL355_Range

.. doxygenenum:: PL::ADXL355_InterruptPolarity

.. doxygenenum:: PL::ADXL355_I2CSpeed

.. doxygenstruct:: PL::ADXL355_DeviceInfo
  :members:
  :protected-members:

.. doxygenstruct:: PL::ADXL355_RawAccelerations
  :members:
  :protected-members:

.. doxygenstruct:: PL::ADXL355_Accelerations
  :members:
  :protected-members: