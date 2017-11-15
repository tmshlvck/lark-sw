/* VARIO
 * LARK
 * TH
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"

#include "ms5611.h"

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO 18
#define BUZZ_GPIO 25

#define I2C_SCL 22
#define I2C_SDA 21

#define PIN_NUM_MISO 14
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK 12
#define PIN_NUM_CS 15

#define STACK_SIZE 2048

void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/*
int dir=1;
int vscm_g=0;
int update_vscm(int vscm) {
	if (vscm > 200)
		dir=-1;
	if (vscm < -200)
		dir=1;
	return vscm+(dir*10);
}

int vario_freq(int vscm)
{
	int out=200;
	out+=vscm*10;
	                    //Initialize the LCD
	if (out > 4000)
		return 4000;
	if (out < 100)
		return 100;
	return out;
}

#define VAR_MAX_T 100
#define VAR_MIN_T 10

int vario_t1(int vscm) {
	int out=VAR_MAX_T;
	out-=vscm*2;
	if (out > VAR_MAX_T)
		return VAR_MAX_T;
	if (out < VAR_MIN_T)
		return VAR_MIN_T;
	return out;
}

int vario_t2(int t1) {
	return VAR_MAX_T-t1;
}



void buzz_task(void *pvParameter)
{
	ledc_timer_config_t ledc_timer = {
		.bit_num = LEDC_TIMER_10_BIT, //set timer counter bit number
		.freq_hz = 440,              //set frequency of pwm
		.speed_mode = LEDC_HIGH_SPEED_MODE, //timer mode,
		.timer_num = LEDC_TIMER_0    //timer index
	};
	ledc_timer_config(&ledc_timer);

	ledc_channel_config_t ledc_channel = {
		.channel = 0,
		.duty = 0,
		.gpio_num = BUZZ_GPIO,
		.intr_type = LEDC_INTR_DISABLE,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.timer_sel = LEDC_TIMER_0
	};
	ledc_channel_config(&ledc_channel);

	while(1) {
		int vscm = vscm_g;
		ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, vario_freq(vscm));

		if(vario_t2(vario_t1(vscm)) > 0) {
			ledc_set_duty(LEDC_HIGH_SPEED_MODE, 0, 0);
			ledc_update_duty(LEDC_HIGH_SPEED_MODE, 0);
			vTaskDelay(vario_t2(vario_t1(vscm)) / portTICK_PERIOD_MS);
		}

		ledc_set_duty(LEDC_HIGH_SPEED_MODE, 0, 0x7F);
		ledc_update_duty(LEDC_HIGH_SPEED_MODE, 0);
		vTaskDelay(vario_t1(vscm) / portTICK_PERIOD_MS);
	}
}
*/

void spi_read(spi_device_handle_t spi) 
{
	esp_err_t ret;
	spi_transaction_t t;
	const unsigned char config[4] = {((1<<7)|(4<<4)|(1<1)), ((4<<5)|(1<<1)|(1)), ((1<<7)|(4<<4)|(1<<1)), ((4<<5)|(1))};
	unsigned char rxb[4];

	memset(&t, 0, sizeof(t));
	t.length=32;
	t.tx_buffer=config;
	t.rx_buffer=rxb;
	//t.user=(void*)0;
	ret=spi_device_transmit(spi, &t);
	assert(ret==ESP_OK);
	printf("Received: 0x%x 0x%x 0x%x 0x%x\n", rxb[0], rxb[1], rxb[2], rxb[3]);
}

void spi_read_task(void *pvParameter) {
	vTaskDelay(100);

	esp_err_t ret;
	spi_device_handle_t spi;
	spi_bus_config_t buscfg={
		.miso_io_num=PIN_NUM_MISO,
		.mosi_io_num=PIN_NUM_MOSI,
		.sclk_io_num=PIN_NUM_CLK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1
	};
        spi_device_interface_config_t devcfg={
		.clock_speed_hz=10000,
		.mode=1,
		.spics_io_num=PIN_NUM_CS, 
		.queue_size=1,
		.command_bits=0,
		.address_bits=0,
		.dummy_bits=0,
		.flags = 0,
		.cs_ena_pretrans=8,
		.cs_ena_posttrans=8,
	};

	ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
	assert(ret==ESP_OK);
	ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
	assert(ret==ESP_OK);

	while (1) {
		spi_read(spi);
		vTaskDelay(10);
	}
}

#define I2C_SCL 22
#define I2C_SDA 21
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0
#define I2C_MASTER_FREQ_HZ         100000

//#define I2C_PRESS_ADDR 0x76
#define I2C_PRESS_ADDR 0x77



void i2c_read_task(void *pvParameter) {
	vTaskDelay(100);

	int i2c_master_port = I2C_NUM_0;
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_SDA;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = I2C_SCL;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	i2c_param_config(i2c_master_port, &conf);
	i2c_driver_install(i2c_master_port, conf.mode,
		I2C_MASTER_RX_BUF_DISABLE,
		I2C_MASTER_TX_BUF_DISABLE, 0);

	ms5611_drv_t dev;
	int ret = ms5611_init(&dev, I2C_NUM_0, I2C_PRESS_ADDR);
	printf("ms5611 init: %d\n", ret);

	float press;
	float temp;
	while(1) {
		press =  ms5611_get_pressure(&dev, MS5611_OSR_DEFAULT);
		printf("read press: %f (%fm)\n", press, ms5611_calc_altitude(&press));
		temp =  ms5611_get_temp(&dev, MS5611_OSR_DEFAULT);
		printf("read temp: %f\n", temp);
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
	
}

void app_main()
{
	xTaskCreate(&blink_task, "blink_task", STACK_SIZE, NULL, 5, NULL);
//	xTaskCreate(&spi_read_task, "spi_read_task", STACK_SIZE, NULL, 5, NULL);
//	xTaskCreate(&buzz_task, "buzz_task", STACK_SIZE, NULL, 5, NULL);
	xTaskCreate(&i2c_read_task, "i2c_read_task", STACK_SIZE, NULL, 5, NULL);
}


