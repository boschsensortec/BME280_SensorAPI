# Change Log
All notable changes to BME280 Sensor API will be documented in this file.

## v3.3.2, 22 Nov 2017
### Changed
	- Linux compatibility issue fixed

## v3.3.1, 07 Nov 2017
### Changed
	- Created the following user APIs which were previously static
	   * bme280_parse_sensor_data
	   * bme280_compensate_data

## v3.3.0, 13 Jul 2017
### Changed
	- Changed macro FLOATING_POINT_REPRESENTATION to BME280_FLOAT_ENABLE
	- Changed member id to dev_id in struct bme280_dev
	- Changed member interface to intf in struct bme280_dev
	- Changed variable length array in bme280_set_regs to fixed length array to allow a max of 10 registers to be written.
	- Fixed bug with shifting in parse_sensor_data
	- Changed macro MACHINE_64_BIT to BME280_64BIT_ENABLE
	- Updated example in the README.md file and added function pointer templates
	
## v3.2.0, 21 Mar 2017
### Changed
	- API for putting sensor into sleep mode changed.
	- Pressure, Temperature out of range data clipped.
	- 64 bit pressure compensation changed.
	
## v3.1.0, 8 Mar 2017
### Added
	- Device settings APIs.
	- Compensations functions, double datatype for Pressure, Temperature and Humidity.
	- Compensations functions, integer datatype 64bit for Pressure.
### Changed
	- Internal functions related to power mode.

## v3.0.0, 17 Feb 2017
### Added
	Below functionalities are supported
	- Init.
	- Soft reset.
	- Power mode.
	- Compensated functions, integer datatype for Pressure, Temperature and Humidity.
	- Get sensor data.

