/* main FreeRTOS entry point
 * Lark - Lightweight ESP32 vario
 *
 * Copyright (C) 2018 Tomas Hlavacek (tomas.hlavacek@akaflieg.tu-darmstadt.de)
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/i2s.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "sdkconfig.h"

#include "ms5611.h"
#include "audiovario.h"




/* globals */
#define STACK_SIZE 2048



float pt=0;
float dpt=0;
float dpt_test_dir = 1;

#define BLINK_GPIO 18
void debug_task(void *pvParameter)
{
    //gpio_pad_select_gpio(BLINK_GPIO); //default?
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while(1) {

	dpt+=dpt_test_dir*0.2;
	if (abs(dpt)>=10)
		dpt_test_dir *= -1;
	audiovario_update(dpt);

	//vTaskDelay(200/portTICK_PERIOD_MS);

        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


#define I2C_SCL 22
#define I2C_SDA 21
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0
#define I2C_MASTER_FREQ_HZ         100000
#define I2C_PRESS_ADDR 0x77
static float compute_dpt(float update) {
	float pt_prev = pt;
	pt = update;
	return 100*(update-pt_prev);
}

void sensor_read_task(void *pvParameter) {
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
		//printf("read press: %f (%fm)\n", press, ms5611_calc_altitude(&press));
		temp =  ms5611_get_temp(&dev, MS5611_OSR_DEFAULT);
		//printf("read temp: %f\n", temp);

		dpt = compute_dpt(press);
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
	
}


#define BUTTON_FNC GPIO_NUM_4
#define BUTTON_VOL GPIO_NUM_5
#define GPIO_INPUT_PIN_SEL (GPIO_SEL_4 | GPIO_SEL_5)
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle ctrl_evt_queue = NULL;
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(ctrl_evt_queue, &gpio_num, NULL);
}

static void init_buttons(void) {
	gpio_config_t io_conf;
	
	io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);

	ctrl_evt_queue = xQueueCreate(10, sizeof(uint32_t));

	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	gpio_isr_handler_add(BUTTON_FNC, gpio_isr_handler, (void*) BUTTON_FNC);
	gpio_isr_handler_add(BUTTON_VOL, gpio_isr_handler, (void*) BUTTON_VOL);
}

static void control_task(void *pvParameter) {
	uint32_t button_num;

	init_buttons();
	audiovario_start();

	while(1) {
		if(xQueueReceive(ctrl_evt_queue, &button_num, portMAX_DELAY)) {
			printf("button %d intr, val: %d\n", button_num, gpio_get_level(button_num));
			audiovario_change_volume(10);
		}
	}
}

void app_main(void ) {
    	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	//wifi_init_softap();
	//xTaskCreate(&networking_task, "networking_task", STACK_SIZE, NULL, 5, NULL);

	xTaskCreate(&debug_task, "debug_task", STACK_SIZE, NULL, 5, NULL);
	//xTaskCreate(&sensor_read_task, "sensor_read_task", STACK_SIZE, NULL, 5, NULL);
	xTaskCreate(&control_task, "control_task", STACK_SIZE, NULL, 10, NULL);
}


/**********************************************/

static void networking_task(void *pvParameters)
{
/*
    while (1) {

        g_rxtx_need_restart = false;

        ESP_LOGI(TAG, "task tcp_conn.");

        //wating for connecting to AP
        xEventGroupWaitBits(tcp_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);

        ESP_LOGI(TAG, "sta has connected to ap.");

        int socket_ret = ESP_FAIL;

        TaskHandle_t tx_rx_task = NULL;

#if EXAMPLE_ESP_TCP_MODE_SERVER
        if (socket_ret == ESP_FAIL) {
            //create tcp socket
            ESP_LOGI(TAG, "tcp_server will start after 3s...");
            vTaskDelay(3000 / portTICK_RATE_MS);
            ESP_LOGI(TAG, "create_tcp_server.");
            socket_ret = create_tcp_server();
        }
#else //EXAMPLE_ESP_TCP_MODE_SERVER
        if (socket_ret == ESP_FAIL) {
            ESP_LOGI(TAG, "tcp_client will start after 20s...");
            vTaskDelay(20000 / portTICK_RATE_MS);
            ESP_LOGI(TAG, "create_tcp_client.");
            socket_ret = create_tcp_client();
        }
#endif
        if (socket_ret == ESP_FAIL) {
            ESP_LOGI(TAG, "create tcp socket error,stop.");
            continue;
        }

        //create a task to tx/rx data

#if EXAMPLE_ESP_TCP_PERF_TX
        if (tx_rx_task == NULL) {
            if (pdPASS != xTaskCreate(&send_data, "send_data", 4096, NULL, 4, &tx_rx_task)) {
                ESP_LOGE(TAG, "Send task create fail!");
            }
        }
#else //EXAMPLE_ESP_TCP_PERF_TX
        if (tx_rx_task == NULL) {
            if (pdPASS != xTaskCreate(&recv_data, "recv_data", 4096, NULL, 4, &tx_rx_task)) {
                ESP_LOGE(TAG, "Recv task create fail!");
            }
        }
#endif
        double bps;

        while (1) {
            g_total_data = 0;
            vTaskDelay(3000 / portTICK_RATE_MS);//every 3s
            bps = (g_total_data * 8.0 / 3.0) / 1000000.0;

            if (g_rxtx_need_restart) {
                printf("send or receive task encoutner error, need to restart\n");
                break;
            }

#if EXAMPLE_ESP_TCP_PERF_TX
            ESP_LOGI(TAG, "tcp send %.2f Mbits per sec!\n", bps);
#if EXAMPLE_ESP_TCP_DELAY_INFO
            ESP_LOGI(TAG, "tcp send packet total:%d  succeed:%d  failed:%d\n"
                     "time(ms):0-30:%d 30-100:%d 100-300:%d 300-1000:%d 1000+:%d\n",
                     g_total_pack, g_send_success, g_send_fail, g_delay_classify[0],
                     g_delay_classify[1], g_delay_classify[2], g_delay_classify[3], g_delay_classify[4]);
#endif //EXAMPLE_ESP_TCP_DELAY_INFO
#else
            ESP_LOGI(TAG, "tcp recv %.2f Mbits per sec!\n", bps);
#endif //EXAMPLE_ESP_TCP_PERF_TX
        }

        close_socket();
    }
*/
    vTaskDelete(NULL);
}

