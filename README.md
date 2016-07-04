
CONTENTS OF THIS FILE
=======================
	* Introduction
	* Version
	* Integration details
	* Driver files information
	* Supported sensor interface
	* Copyright

INTRODUCTION
===============
	- This package contains the Bosch Sensortec MEMS humidity sensor driver(sensor API)
	- The sensor driver package includes bme280.h, bme280.c and bme280_support.c files
	
VERSION
=========
	- Version of bme280 sensor driver is:
		* bme280.c - V2.0.5
		* bme280.h - V2.0.5
		* bme280_support.c - V1.0.6

INTEGRATION DETAILS
=====================
	- Integrate bme280.h and bme280.c file in to your project.
	- The bme280_support.c file contains only examples for API use cases, so it is not required to integrate into project.

DRIVER FILES INFORMATION
===========================
	bme280.h
	-----------
		* This header file has the register address definition, constant definitions, data type definition and supported sensor driver calls declarations.

	 bme280.c
	------------
		* This file contains the implementation for the sensor driver APIs.

	 bme280_support.c
	----------------------
		* This file shall be used as an user guidance, here you can find samples of
    			* Initialize the sensor with I2C/SPI communication
        				- Add your code to the SPI and/or I2C bus read and bus write functions.
            					- Return value can be chosen by yourself
           					- API just passes that value to your application code
        				- Add your code to the delay function
        				- Change I2C address accordingly in bme280.h
   			* Power mode configuration of the sensor
   			* Get and set functions usage
			* Reading the sensor read out data

SUPPORTED SENSOR INTERFACE
====================================
	- This humidity sensor driver supports SPI and I2C interfaces


COPYRIGHT
===========
	- Copyright (C) 2013 - 2016 Bosch Sensortec GmbH


	


	
