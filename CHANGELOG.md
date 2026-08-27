# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]
### Fixed
- clearFifo reading up to 288 bytes in a single SPI transaction.

## [1.4.1] - 2026-08-20
### Fixed
- ADXL355_InterruptPolarity documentation.

## [1.4.0] - 2026-08-19
### Changed
- Array arguments passed as &array.

### Added
- Tests for getStatus, clearFifo, getAccelerations, getTemperature, getRawAccelerationsFromFifo and getAccelerationsFromFifo.
- Timeout to getRawAccelerationsFromFifo and getAccelerationsFromFifo.

### Removed
- Redundant standby-mode acceleration read in selfTest.

### Fixed
- selfTest not restoring range and not clearing FIFO.
- FIFO reading algorithm.
- clearFifo possible infinite loop.
- getAccelerationScaleFactor not returning NAN for an unexpected range value.
- selfTest not setting a specific output data rate.
- setRawOffsets and setRawActivityDetectionThreshold not clamping the arguments.

## [1.3.0] - 2025-08-21
### Added
- Custom SPI bus selection.

## [1.2.0] - 2025-04-17
### Added
- Get shadow registers function.

## [1.1.0] - 2025-01-14
### Added
- I2C support.

## [1.0.2] - 2024-08-23
### Fixed
- ADXL355::setSynchronization.

## [1.0.1] - 2023-08-28
### Fixed
- Changed include to uppercase.