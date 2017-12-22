/* VARIO
 * LARK
 * TH
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



#define BLINK_GPIO 18

#define I2C_SCL 22
#define I2C_SDA 21

#define STACK_SIZE 2048

/* globals */
char audiovario_running = 0;
char audiovario_stop = 0;



float pt=0;
float dpt=0;
float dpt_test_dir = 1;



void debug_task(void *pvParameter)
{
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    printf("long long size=%d\n", sizeof(long long));
    printf("SAMPLE_MAX=%d\n", SAMPLE_MAX);

    while(1) {
	//printf("dpt=%f\n", dpt);

	dpt+=dpt_test_dir*0.2;
	if (abs(dpt)>=10)
		dpt_test_dir *= -1;
	if (audiovario_running) {
		audiovario_update(dpt);
	}

	vTaskDelay(200/portTICK_PERIOD_MS);

	/*
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
	*/
    }
    while(1)
        vTaskDelay(1000 / portTICK_PERIOD_MS);
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
		dpt = 1.5;
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
	
}

#define AUDIO_I2S_NUM 0
#define AUDIO_BUFFER_SIZE 8192
uint16_t samples_data[AUDIO_BUFFER_SIZE];
void audiovario_task(void *pvParameter) {
	vTaskDelay(200);

	i2s_config_t i2s_config = {
		.mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
		.sample_rate =  SAMPLE_RATE,
		.bits_per_sample = SAMPLE_BITS,
		.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        	.communication_format = I2S_COMM_FORMAT_I2S_MSB,
		.dma_buf_count = 4,
        	.dma_buf_len = 1024,
        	.use_apll = 0,
	       	.intr_alloc_flags = 0
	};
	i2s_driver_install(AUDIO_I2S_NUM, &i2s_config, 0, NULL);
	i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);
    	i2s_set_clk(AUDIO_I2S_NUM, SAMPLE_RATE, SAMPLE_BITS, I2S_CHANNEL_MONO);

	audiovario_init();
	audiovario_running = 1;

	while (!audiovario_stop) {
		audiovario_synthesise(samples_data, AUDIO_BUFFER_SIZE);
		i2s_write_bytes(AUDIO_I2S_NUM, (const char *)samples_data,
				AUDIO_BUFFER_SIZE*sizeof(uint16_t), portMAX_DELAY);
		vTaskDelay(10/portTICK_PERIOD_MS);
	}

	audiovario_running = 0;

	vTaskDelete(NULL);
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
//	xTaskCreate(&sensor_read_task, "sensor_read_task", STACK_SIZE, NULL, 5, NULL);
	xTaskCreate(&audiovario_task, "audiovario_task", STACK_SIZE, NULL, 5, NULL);
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

