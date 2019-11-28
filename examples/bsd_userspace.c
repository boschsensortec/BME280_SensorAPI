/*
 * BSD userspace test code, simple and mose code directy from the doco.
 * compile like this: cc bsd_userspace.c ../bme280.c -I ../ -o bme280
 * tested: NanoPi NEO.
 * Use like: ./bme280 /dev/iic0
 */
#include "bme280.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include <dev/iicbus/iic.h>

int fd;

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void user_delay_ms(uint32_t period);
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void print_sensor_data(struct bme280_data *comp_data);
int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev);


int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    struct iic_msg msgs[2] = {
        {id << 1 | IIC_M_WR, IIC_M_WR, 1, &reg_addr},
        {id << 1 | IIC_M_RD, IIC_M_RD, len, data},
    };

    struct iic_rdwr_data rdwr_data = {msgs, 2};

    int error = ioctl(fd, I2CRDWR, &rdwr_data);
    if (error) {
        return BME280_E_COMM_FAIL;
    }

    return BME280_OK;
}


void user_delay_ms(uint32_t period)
{
    usleep(period * 1000);
}


int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    uint8_t *buf = malloc( (1 + len) * sizeof(uint8_t) );
    if (buf == NULL) {
        return BME280_E_COMM_FAIL;
    }

    buf[0] = reg_addr;

    for (int i = 0; i < len; i++) {
        buf[i+1] = data[i];
    }

    struct iic_msg msg;

    msg.slave = id << 1 | !IIC_M_RD;
    msg.flags = !IIC_M_RD;
    msg.len = (1 + len) * sizeof(uint8_t);
    msg.buf = buf;

    struct iic_rdwr_data rdwr_data = {&msg, 1};

    int error = ioctl(fd, I2CRDWR, &rdwr_data);
    if (error) {
        free(buf);
        return BME280_E_COMM_FAIL;
    }

    free(buf);

    return BME280_OK;
}


void print_sensor_data(struct bme280_data *comp_data)
{
    float temp, press, hum;
#ifdef BME280_FLOAT_ENABLE
    temp = comp_data->temperature;
    press = 0.01 * comp_data->pressure;
    hum = comp_data->humidity;
#else
#ifdef BME280_64BIT_ENABLE
    temp = 0.01f * comp_data->temperature;
    press = 0.0001f * comp_data->pressure;
    hum = 1.0f / 1024.0f * comp_data->humidity;
#else
    temp = 0.01f * comp_data->temperature;
    press = 0.01f * comp_data->pressure;
    hum = 1.0f / 1024.0f * comp_data->humidity;
#endif
#endif
    printf("%0.2lf deg C, %0.2lf hPa, %0.2lf%%\n", temp, press, hum);
}


int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t settings_sel;
    struct bme280_data comp_data;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, dev);
    if (rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to set sensor settings (code %+d).", rslt);
        return rslt;
    }

    printf("Temperature, Pressure, Humidity\n");
    /* Continuously stream sensor data */
    while (1)
    {
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
        if (rslt != BME280_OK)
        {
            fprintf(stderr, "Failed to set sensor mode (code %+d).", rslt);
            break;
        }
        /* Wait for the measurement to complete and print data @25Hz */
        dev->delay_ms(40);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
        if (rslt != BME280_OK)
        {
            fprintf(stderr, "Failed to get sensor data (code %+d).", rslt);
            break;
        }
        print_sensor_data(&comp_data);
        dev->delay_ms(1000);
    }

    return rslt;
}



int main(int argc, char* argv[])
{
    struct bme280_dev dev;
    int8_t rslt = BME280_OK;

    if (argc < 2)
    {
        fprintf(stderr, "Missing argument for i2c bus.\n");
        exit(1);
    }

    // make sure to select BME280_I2C_ADDR_PRIM
    // or BME280_I2C_ADDR_SEC as needed
    dev.dev_id =
#if 1
        BME280_I2C_ADDR_PRIM
#else
        BME280_I2C_ADDR_SEC
#endif
;

    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;


    if ((fd = open(argv[1], O_RDWR)) < 0)
    {
        fprintf(stderr, "Failed to open the i2c bus %s\n", argv[1]);
        exit(1);
    }

    rslt = bme280_init(&dev);
    if (rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to initialize the device (code %+d).\n", rslt);
        exit(1);
    }

    rslt = stream_sensor_data_forced_mode(&dev);
    if (rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to stream sensor data (code %+d).\n", rslt);
        exit(1);
    }
    return 0;
}
