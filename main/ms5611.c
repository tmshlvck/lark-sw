/* MS5611
 * LARK
 * TH
 * Inspired by: Crazyflie-Firmware, thanks!!!
 */


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "ms5611.h"
#include "math.h"


/* i2c abstraction */

#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

#define I2C_TIMEOUT (1000 / portTICK_RATE_MS)

static int ms2tick(int ms) {
	if (ms < portTICK_PERIOD_MS)
		return 1;
	else
		return ms / portTICK_PERIOD_MS;
}

static esp_err_t ms5611_i2c_cmd(i2c_port_t i2c_num, uint8_t addr, uint8_t command)
{
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, addr << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, command, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_TIMEOUT);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
		return ret;
	}
	return ret;
}


static esp_err_t ms5611_i2c_write_byte(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, uint8_t data)
{
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, addr << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, data, ACK_CHECK_DIS);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_TIMEOUT);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
		return ret;
	}
	return ret;
}


static esp_err_t ms5611_i2c_read_byte(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, uint8_t* data)
{
	int ret;
	ret = ms5611_i2c_cmd(i2c_num, addr, reg);
	if (ret != ESP_OK) {
		return ret;
	}
//	vTaskDelay(ms2tick(30));

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, addr | READ_BIT, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, data, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_TIMEOUT);
	i2c_cmd_link_delete(cmd);
	return ret;
}


static esp_err_t ms5611_i2c_read(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, size_t size, uint8_t* data)
{
	int ret;
	ret = ms5611_i2c_cmd(i2c_num, addr, reg);
	if (ret != ESP_OK) {
		return ret;
	}
//	vTaskDelay(ms2tick(30));

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, addr << 1 | READ_BIT, ACK_CHECK_EN);
	if (size>1)
		i2c_master_read(cmd, data, size-1, ACK_VAL);
	i2c_master_read_byte(cmd, data+size-1, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_TIMEOUT);
	i2c_cmd_link_delete(cmd);
	return ret;
}




/* ms5611 driver */


#define EXTRA_PRECISION      5 // trick to add more precision to the pressure and temp readings
#define CONVERSION_TIME_MS   10 // conversion time in milliseconds. 10 is minimum
#define PRESSURE_PER_TEMP 5 // Length of reading cycle: 1x temp, rest pressure. Good values: 1-10
#define FIX_TEMP 25         // Fixed Temperature. ASL is a function of pressure and temperature, but as the temperature changes so much (blow a little towards the flie and watch it drop 5 degrees) it corrupts the ASL estimates.
                            // TLDR: Adjusting for temp changes does more harm than good.

/**
 * Reads factory calibration and store it into object variables.
 */
static int ms5611_read_PROM(ms5611_drv_t *dev)
{
	uint8_t buffer[MS5611_PROM_REG_SIZE];
	uint16_t* cru16 = (uint16_t*)&dev->cr;
	int status;

	for (int i = 0; i < MS5611_PROM_REG_COUNT; i++) {
		status = ms5611_i2c_read(dev->i2c_num, dev->addr, (MS5611_PROM_BASE_ADDR + (i * MS5611_PROM_REG_SIZE)), MS5611_PROM_REG_SIZE, buffer);
		cru16[i] = ((uint16_t)buffer[0] << 8) | buffer[1];
		if (status)
			return status;
	}

	printf("DEBUG ms5611 cr psens=%d\n", dev->cr.psens);
	printf("DEBUG ms5611 cr off=%d\n", dev->cr.off);
	printf("DEBUG ms5611 cr tcs=%d\n", dev->cr.tcs);
	printf("DEBUG ms5611 cr tco=%d\n", dev->cr.tco);
	printf("DEBUG ms5611 cr tref=%d\n", dev->cr.tref);
	printf("DEBUG ms5611 cr tsens=%d\n", dev->cr.tsens);

	return status;
}

/**
 * Send a reset command to the device. With the reset command the device
 * populates its internal registers with the values read from the PROM.
 */
void ms5611_reset(ms5611_drv_t *dev)
{
	ms5611_i2c_cmd(dev->i2c_num, dev->addr, MS5611_RESET);
}

#define MS5611_RESET_TIME 5

int ms5611_init(ms5611_drv_t *dev, i2c_port_t i2c_num, uint8_t addr)
{
	if (dev->initialized)
		return 0;

	dev->i2c_num = i2c_num;
	dev->addr = addr;

	ms5611_reset(dev); // reset the device to populate its internal PROM registers
	vTaskDelay(ms2tick(MS5611_RESET_TIME));
	if (ms5611_read_PROM(dev)) {
		return -1;
	}

	dev->initialized = 1;
	return 0;
}

static void ms5611_start_conv(ms5611_drv_t *dev, int8_t command)
{
	ms5611_i2c_cmd(dev->i2c_num, dev->addr, command);
}

void ms5611_start_conv_press(ms5611_drv_t *dev) {
	ms5611_start_conv(dev, MS5611_D1 + MS5611_OSR_DEFAULT);
}

void ms5611_start_conv_temp(ms5611_drv_t *dev) {
	ms5611_start_conv(dev, MS5611_D2 + MS5611_OSR_DEFAULT);
}

int32_t ms5611_get_conv(ms5611_drv_t *dev)
{
	int32_t conversion = 0;
	uint8_t buffer[MS5611_D1D2_SIZE];

	ms5611_i2c_read(dev->i2c_num, dev->addr, 0, MS5611_D1D2_SIZE, buffer);
	conversion = ((int32_t)buffer[0] << 16) |
		((int32_t)buffer[1] << 8) | buffer[2];

	return conversion;
}

int32_t ms5611_get_deltatemp(ms5611_drv_t *dev, int32_t rawtemp)
{
	if (rawtemp != 0)
		return rawtemp - (((int32_t)dev->cr.tref) << 8);
	else
		return 0;
}

float ms5611_get_temp(ms5611_drv_t *dev, int32_t rawtemp)
{
	/* see datasheet page 7 for formulas */
	int32_t dT = ms5611_get_deltatemp(dev, rawtemp);
	if (dT != 0)
    		return (float)(((1 << EXTRA_PRECISION) * 2000)
			+ (((int64_t)dT * dev->cr.tsens) >> (23 - EXTRA_PRECISION)))
			/ ((1 << EXTRA_PRECISION)* 100.0);

	else
		return 0;
}

float ms5611_get_pressure(ms5611_drv_t *dev, int32_t rawpress, int32_t rawtemp)
{
	/* see datasheet page 7 for formulas */
	int64_t dT = (int64_t)ms5611_get_deltatemp(dev, rawtemp);
	if (dT == 0)
		return 0;

	int64_t off = (((int64_t)dev->cr.off) << 16) + ((dev->cr.tco * dT) >> 7);
	int64_t sens = (((int64_t)dev->cr.psens) << 15) + ((dev->cr.tcs * dT) >> 8);
	if (rawpress != 0)
		return ((((rawpress * sens) >> 21) - off) >> (15 - EXTRA_PRECISION))
			/ ((1 << EXTRA_PRECISION) * 100.0);
	else
		return 0;
}


/**
 * Converts pressure to altitude above sea level (ASL) in meters
*/
float ms5611_calc_altitude(float pressure/*, float* ground_pressure, float* ground_temp*/)
{
	if (pressure > 0) {
		//return (1.f - pow(pressure / CONST_SEA_PRESSURE, CONST_PF)) * CONST_PF2;
		//return ((pow((1015.7 / pressure), CONST_PF) - 1.0) * (25. + 273.15)) / 0.0065;
		return ((pow((1015.7 / pressure), CONST_PF) - 1.0) * (FIX_TEMP + 273.15)) / 0.0065;
	} else
		return 0;
}


