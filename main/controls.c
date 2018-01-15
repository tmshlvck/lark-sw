/* Lark controls 
 * Copyright (C) 2018 Tomas Hlavacek (tomas.hlavacek@akaflieg.tu-darmstadt.de)
 *
 * This file is part of Lark.
 *
 * Lark is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Lark is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Lark.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_err.h"

#include "sdkconfig.h"

#include "audiovario.h"
#include "controls.h"


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

void control_task(void *pvParameter) {
	uint32_t button_num;

	init_buttons();
	audiovario_start();

	int voldir=100;

	while(1) {
		if(xQueueReceive(ctrl_evt_queue, &button_num, portMAX_DELAY)) {
			printf("button %d intr, val: %d\n", button_num, gpio_get_level(button_num));
			audiovario_change_volume(voldir);
			voldir*=-1;
		}
	}
}

