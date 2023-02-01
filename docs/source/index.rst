Analog Devices ADXL355 Accelerometer Library for Arduino
========================================================
Tested on `Arduino Due <https://docs.arduino.cc/hardware/due>`_ and
`EVAL-ADXL355-PMDZ <https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-ADXL355-PMDZ.html>`_

Examples
--------
| `Device information and self test <https://github.com/plasmapper/adxl355-arduino/tree/main/examples/DeviceInfoAndSelfTest>`_
| `Acceleration measurement <https://github.com/plasmapper/adxl355-arduino/tree/main/examples/AccelerationMeasurement>`_
| `Acceleration waveform measurement <https://github.com/plasmapper/adxl355-arduino/tree/main/examples/AccelerationWaveformMeasurement>`_
| `Activity detection <https://github.com/plasmapper/adxl355-arduino/tree/main/examples/ActivityDetection>`_

Functions
---------

Initialization
^^^^^^^^^^^^^^
:cpp:func:`PL::ADXL355::ADXL355`, :cpp:func:`PL::ADXL355::begin`.

Range Configuration and Scale Factor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::ADXL355::getRange`, :cpp:func:`PL::ADXL355::setRange`, :cpp:func:`PL::ADXL355::getAccelerationScaleFactor`.

Measurement Frequency Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::ADXL355::getOutputDataRate`, :cpp:func:`PL::ADXL355::setOutputDataRate`.

Measurement Enable/Disable
^^^^^^^^^^^^^^^^^^^^^^^^^^
Use :cpp:func:`PL::ADXL355::enableMeasurement`, :cpp:func:`PL::ADXL355::disableMeasurement`,
:cpp:func:`PL::ADXL355::isMeasurementEnabled`.

Single Measurement
^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::ADXL355::getAccelerations`/:cpp:func:`PL::ADXL355::getRawAccelerations`.

Waveform Measurement
^^^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::ADXL355::getNumberOfFifoSamples`, :cpp:func:`PL::ADXL355::clearFifo`,
:cpp:func:`PL::ADXL355::getAccelerationsFromFifo`/:cpp:func:`PL::ADXL355::getRawAccelerationsFromFifo`.

Offset Configuration
^^^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::ADXL355::getOffsets`/:cpp:func:`PL::ADXL355::getRawOffsets`,
:cpp:func:`PL::ADXL355::setOffsets`/:cpp:func:`PL::ADXL355::setRawOffsets`.

Activity Detection
^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::ADXL355::getActivityDetectionAxes`, :cpp:func:`PL::ADXL355::setActivityDetectionAxes`,
:cpp:func:`PL::ADXL355::getActivityDetectionThreshold`/:cpp:func:`PL::ADXL355::getRawActivityDetectionThreshold`,
:cpp:func:`PL::ADXL355::setActivityDetectionThreshold`/:cpp:func:`PL::ADXL355::setRawActivityDetectionThreshold`,
:cpp:func:`PL::ADXL355::getActivityDetectionCount`, :cpp:func:`PL::ADXL355::setActivityDetectionCount`.

Temperature Measurement
^^^^^^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::ADXL355::enableTemperature`, :cpp:func:`PL::ADXL355::disableTemperature`,
:cpp:func:`PL::ADXL355::isTemperatureEnabled`,
:cpp:func:`PL::ADXL355::getTemperature`/:cpp:func:`PL::ADXL355::getRawTemperature`.

Interrupt Configuration
^^^^^^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::ADXL355::getInterrupts`, :cpp:func:`PL::ADXL355::setInterrupts`.

Device Information and Status
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::ADXL355::getDeviceInfo`, :cpp:func:`PL::ADXL355::getStatus`.

Self-test and Reset
^^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::ADXL355::selfTest`, :cpp:func:`PL::ADXL355::reset`.

Other
^^^^^
:cpp:func:`PL::ADXL355::getHpfFrequency`, :cpp:func:`PL::ADXL355::setHpfFrequency`,
:cpp:func:`PL::ADXL355::getFifoWatermark`, :cpp:func:`PL::ADXL355::setFifoWatermark`,
:cpp:func:`PL::ADXL355::getSynchronization`, :cpp:func:`PL::ADXL355::setSynchronization`,
:cpp:func:`PL::ADXL355::enableExternalClock`, :cpp:func:`PL::ADXL355::disableExternalClock`,
:cpp:func:`PL::ADXL355::isExternalClockEnabled`
:cpp:func:`PL::ADXL355::getInterruptPolarity`, :cpp:func:`PL::ADXL355::setInterruptPolarity`,
:cpp:func:`PL::ADXL355::getI2CSpeed`, :cpp:func:`PL::ADXL355::setI2CSpeed`,
:cpp:func:`PL::ADXL355::enableDataReady`, :cpp:func:`PL::ADXL355::disableDataReady`,
:cpp:func:`PL::ADXL355::isDataReadyEnabled`.


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