# BME280 sensor API
## Introduction
This package contains the Bosch Sensortec's BME280 pressure sensor driver (sensor API)

The sensor driver package includes bme280.c, bme280.h and bme280_defs.h files.

## Version
File | Version | Date
-----|---------|-----
bme280.c |  3.2.0     | 21 Mar 2017
bme280.h |  3.2.0     | 21 Mar 2017
bme280_defs.h |  3.2.0     | 21 Mar 2017

## Integration details
* Integrate bme280.h, bme280_defs.h and bme280.c file in to the project.
* Include the bme280.h file in your code like below.
``` c
#include "bme280.h"
```

## File information
* bme280_defs.h : This header file has the constants, macros and datatype declarations.
* bme280.h : This header file contains the declarations of the sensor driver APIs.
* bme280.c : This source file contains the definitions of the sensor driver APIs.

## Supported sensor interfaces
* SPI 4-wire
* I2C

SPI 3-wire is currently not supported in the API.
## Usage guide
### Initializing the sensor
To initialize the sensor, user need to create a device structure. User can do this by 
creating an instance of the structure bme280_dev. After creating the device strcuture, user 
need to fill in the various parameters as shown below.

#### Example for SPI 4-Wire
``` c
struct bme280_dev dev;
int8_t rslt = BME280_OK;

/* Sensor_0 interface over SPI with native chip select line */
dev.id = 0;
dev.interface = BME280_SPI_INTF;
dev.read = user_spi_read;
dev.write = user_spi_write;
dev.delay_ms = user_delay_ms;

rslt = bme280_init(&dev);
```
#### Example for I2C
``` c
struct bme280_dev dev;
int8_t rslt = BME280_OK;

dev.id = BME280_I2C_ADDR_PRIM;
dev.interface = BME280_I2C_INTF;
dev.read = user_i2c_read;
dev.write = user_i2c_write;
dev.delay_ms = user_delay_ms;

rslt = bme280_init(&dev);
```
Regarding compensation functions for temperature,pressure and humidity we have two implementations.
1) Double precision floating point version
2) Integer version

By default, integer version is used in the API. If user needs double version, user has to
enable FLOATING_POINT_REPRESENTATION macro in bme280_defs.h file.

In integer compensation functions, we also have below two implementations for pressure.
1) For 32 bit machine.
2) For 64 bit machine.

By default, 64 bit variant is used in the API. If user wants 32 bit variant, user can disable the
macro MACHINE_64_BIT in bme280_defs.h file.

### Get sensor data
#### Get sensor data in forced mode

``` c
int8_t get_sensor_data_forced_mode(struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t settings_sel;
	struct bme280_data comp_data;

	/* Continuously get the sensor data */
	while (1) {
		dev->settings.osr_h = BME280_OVERSAMPLING_4X;
		dev->settings.osr_p = BME280_OVERSAMPLING_4X;
		dev->settings.osr_t = BME280_OVERSAMPLING_4X;

		settings_sel = BME280_OSR_PRESS_SEL|BME280_OSR_TEMP_SEL|BME280_OSR_HUM_SEL;

		rslt = bme280_set_sensor_settings(settings_sel, dev);
		rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
		/* Give some delay for the sensor to go into force mode */
		dev->delay_ms(5);
		rslt = bme280_get_sensor_data(BME280_PRESS | BME280_HUM | BME280_TEMP, &comp_data, dev);
		print_sensor_data(&comp_data);
	}
	return rslt;
}

void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef FLOATING_POINT_REPRESENTATION
		printf("%0.2f\t\t%0.2f\t\t%0.2f\t\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
		printf("%ld\t\t%ld\t\t%ld\t\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}

```
##### Get sensor data in normal mode
``` c
int8_t get_sensor_data_normal_mode(struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t settings_sel;
	struct bme280_data comp_data;

	dev->settings.osr_h = BME280_OVERSAMPLING_4X;
	dev->settings.osr_p = BME280_OVERSAMPLING_4X;
	dev->settings.osr_t = BME280_OVERSAMPLING_4X;

	settings_sel = BME280_OSR_PRESS_SEL|BME280_OSR_TEMP_SEL|BME280_OSR_HUM_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);
	/* Give some delay for the sensor to go into normal mode */
	dev->delay_ms(5);
	
	while (1) {
		rslt = bme280_get_sensor_data(BME280_PRESS | BME280_HUM | BME280_TEMP, &comp_data, dev);
		print_sensor_data(&comp_data);
	}

	return rslt;
}

void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef FLOATING_POINT_REPRESENTATION
		printf("%0.2f\t\t%0.2f\t\t%0.2f\t\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
		printf("%ld\t\t%ld\t\t%ld\t\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}
```

## Copyright (C) 2016 - 2017 Bosch Sensortec GmbH