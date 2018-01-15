/* Lark Audiovario Sythesis
 * Copyright (C) 2018 Tomas Hlavacek (tomas.hlavacek@akaflieg.tu-darmstadt.de)
 *
 * Inspired by OpenVario (https://www.openvario.org) project.
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

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2s.h"

#include "audiovario.h" 


typedef struct audiovario_config {
	float deadband_low;
	float deadband_high;
	int pulse_length;
	float pulse_length_gain;
	int pulse_duty;
	int pulse_rise;
	int pulse_fall;
	int base_freq_pos;
	int base_freq_neg;
	float freq_gain_pos;
	float freq_gain_neg;
} audiovario_config_t;



/* global config */
audiovario_config_t vario_config[2];
int volume = 0;
audiovario_mode_t vario_mode = vario;
SemaphoreHandle_t audiovario_mutex;
int audiovario_running = 0;

/* global status */
uint16_t vario_last = 0;
int vario_dir = 1;
int pulse_phase = 0;
int freq = 0;
int samples_per_pulse = 0;
int vol_max = 0;



static void audiovario_init_synth(void) {
	volume = 0;

	vario_config[vario].deadband_low = DEADBAND_LOW;
	vario_config[vario].deadband_high = DEADBAND_HIGH;
	vario_config[vario].pulse_length = PULSE_LENGTH;
	vario_config[vario].pulse_length_gain = PULSE_LENGTH_GAIN;
	vario_config[vario].pulse_duty = PULSE_DUTY;
	vario_config[vario].pulse_rise = PULSE_RISE;
	vario_config[vario].pulse_fall = PULSE_FALL;
	vario_config[vario].base_freq_pos = BASE_FREQ_POS;
	vario_config[vario].base_freq_neg = BASE_FREQ_NEG;
	vario_config[vario].freq_gain_pos = FREQ_GAIN_POS;
	vario_config[vario].freq_gain_neg = FREQ_GAIN_NEG;
	
	vario_config[stf].deadband_low = STF_DEADBAND_LOW;
	vario_config[stf].deadband_high = STF_DEADBAND_HIGH;
	vario_config[stf].pulse_length = STF_PULSE_LENGTH;
	vario_config[stf].pulse_length_gain = STF_PULSE_LENGTH_GAIN;
	vario_config[stf].pulse_duty = STF_PULSE_DUTY;
	vario_config[stf].pulse_rise = STF_PULSE_RISE;
	vario_config[stf].pulse_fall = STF_PULSE_FALL;
	vario_config[stf].base_freq_pos = STF_BASE_FREQ_POS;
	vario_config[stf].base_freq_neg = STF_BASE_FREQ_NEG;
	vario_config[stf].freq_gain_pos = STF_FREQ_GAIN_POS;
	vario_config[stf].freq_gain_neg = STF_FREQ_GAIN_NEG;

	audiovario_mutex = xSemaphoreCreateMutex();

	audiovario_set_mode(vario);
	audiovario_update(0);
}

void audiovario_set_mode(audiovario_mode_t mode) {
	vario_mode=mode;
}

int audiovario_change_volume(int delta) {
	volume+=delta;
	if (volume<0) volume=0;
	if (volume>100) volume=100;
	return volume;
}

int audiovario_set_volume(int vol) {
	volume=vol;
	if (volume<0) volume=0;
	if (volume>100) volume=100;
	return volume;
}

#ifdef DEBUG
static void print_buffer(uint16_t *buf, size_t frames) {
	for (int i=0; i<frames; i++) {
		printf("%d : %d\n", i, buf[i]);
	}
}
#endif


void audiovario_update(float val) {
	audiovario_config_t *conf = &vario_config[vario_mode];

	if ((volume == 0) || (!audiovario_running))
		return;

	if (xSemaphoreTake(audiovario_mutex, portMAX_DELAY) != pdTRUE)
		printf("semaphore fail!\n");
	if ((val > conf->deadband_low) && (val < conf->deadband_high)) {
		freq = 0;
		xSemaphoreGive(audiovario_mutex);
		return;
	}

	if (val > 0){
		freq = conf->base_freq_pos + (val * conf->freq_gain_pos);
		if (val > 0.5)
			samples_per_pulse = conf->pulse_length/(val*conf->pulse_length_gain);
		else
			samples_per_pulse = conf->pulse_length * 2;
	} else {
		freq = conf->base_freq_neg / (1.0 - (val * conf->freq_gain_neg));
		samples_per_pulse = 0;
	}

	xSemaphoreGive(audiovario_mutex);
}

static int pulse_volume_max(void) {
	audiovario_config_t *conf = &vario_config[vario_mode];
	int l_vol_max = (AUDIO_SAMPLE_MAX * volume) / 100;
	unsigned int samples_on_per_pulse = samples_per_pulse * conf->pulse_duty / 100;

	if (samples_per_pulse <= 0) /* no pulses */
		return l_vol_max;
	
	if (pulse_phase < samples_on_per_pulse) { /* pulses */
		if ((pulse_phase + 1) * 100 < conf->pulse_rise * samples_on_per_pulse) /* fade in */
			return (((unsigned long long)l_vol_max) * (pulse_phase + 1) * 100) /
				(conf->pulse_rise * samples_on_per_pulse);
		else /* main run */
			return l_vol_max;
	} else {
		if ((pulse_phase - samples_on_per_pulse + 1) * 100 <
				conf->pulse_fall * samples_on_per_pulse) /* fade out */
			return l_vol_max - ((((unsigned long long)l_vol_max) *
				(pulse_phase - samples_on_per_pulse + 1) * 100) /
				(conf->pulse_rise * samples_on_per_pulse));

		else /* silent period */
			return 0;
	}
}

static void audiovario_synthesise(uint16_t *buffer, size_t frames) {
	int sample;
	
	if (xSemaphoreTake(audiovario_mutex, portMAX_DELAY) != pdTRUE)
		printf("semaphore fail!\n");

	if ((volume == 0) || (freq <= 0)) {
		for (size_t i=0; i<frames; i++)
			buffer[i] = vario_last;
		xSemaphoreGive(audiovario_mutex);
		return;
	}

	for (size_t i=0; i<frames; i++) {
		if ((vario_last == 0) || (vol_max == 0))
			vol_max = pulse_volume_max();

		if (vol_max == 0) {
			sample = vario_last; /* silence */
		} else { /* triangles */
			sample = vario_last + ((vol_max * vario_dir * freq) / AUDIO_SAMPLE_RATE);
			if (sample >= vol_max) {
				sample = vol_max;
				vario_dir *= -1;
			}
			if (sample <= 0) {
				sample = 0;
				vario_dir *= -1;
			}
		}

		/* update global status */
		buffer[i] = vario_last = sample;
		pulse_phase++;
		if (pulse_phase > samples_per_pulse)
			pulse_phase = 0;
	}

	xSemaphoreGive(audiovario_mutex);
}

static void audiovario_task(void *pvParameter) {
	uint16_t *samples_data = malloc(AUDIO_BUFFER_SIZE * sizeof(uint16_t));

	vTaskDelay(200);

	audiovario_running = 1;

	while (audiovario_running) {
		audiovario_synthesise(samples_data, AUDIO_BUFFER_SIZE);
		i2s_write_bytes(AUDIO_I2S_NUM, (const char *)samples_data,
				AUDIO_BUFFER_SIZE*sizeof(uint16_t), portMAX_DELAY);
		vTaskDelay(AUDIO_TASK_PERIOD_MS/portTICK_PERIOD_MS);
	}

	free(samples_data);

	i2s_driver_uninstall(AUDIO_I2S_NUM);
	vTaskDelete(NULL);
}

BaseType_t audiovario_start(void) {
	if (audiovario_running)
		return NULL;

	i2s_config_t i2s_config = {
		.mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
		.sample_rate =  AUDIO_SAMPLE_RATE,
		.bits_per_sample = AUDIO_SAMPLE_BITS,
		.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        	.communication_format = I2S_COMM_FORMAT_I2S_MSB,
		.dma_buf_count = 4,
        	.dma_buf_len = 1024,
        	.use_apll = 0,
		.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1
	};

	esp_err_t err = i2s_driver_install(AUDIO_I2S_NUM, &i2s_config, 0, NULL);
	if (err != ESP_OK) {
		printf("i2s_driver_install failed\n");
		return err;
	}

	i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);
    	i2s_set_clk(AUDIO_I2S_NUM, AUDIO_SAMPLE_RATE, AUDIO_SAMPLE_BITS, I2S_CHANNEL_MONO);

	audiovario_init_synth();

	return xTaskCreate(&audiovario_task, "audiovario_task", AUDIO_STACK_SIZE, NULL, 5, NULL);
}

void audiovario_stop(void) {
	audiovario_running = 0;
}


