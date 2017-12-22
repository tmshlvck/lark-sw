/* Project Lark by Tomas Hlavacek (tmshlvck@gmail.com)
 * Inspired by OpenVario. Many thanks!!!
 */

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "audiovario.h" 


/* global config */
audiovario_config_t vario_config[2];
int volume = 0;
audiovario_mode_t vario_mode = vario;
SemaphoreHandle_t audiovario_mutex;

void audiovario_init(void) {
	volume = 100;

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


static void print_buffer(uint16_t *buf, size_t frames) {
	for (int i=0; i<frames; i++) {
		printf("%d : %d\n", i, buf[i]);
	}
}



/* global status */
uint16_t vario_last = 0;
int vario_dir = 1;
int pulse_phase = 0;
int freq = 0;
int samples_per_pulse = 0;

void audiovario_update(float val) {
	audiovario_config_t *conf = &vario_config[vario_mode];

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
	int vol_max = (SAMPLE_MAX * volume) / 100;
	unsigned int samples_on_per_pulse = samples_per_pulse * conf->pulse_duty / 100;

	if (samples_per_pulse <= 0) /* no pulses */
		return vol_max;
	
	if (pulse_phase < samples_on_per_pulse) { /* pulses */
		if ((pulse_phase + 1) * 100 < conf->pulse_rise * samples_on_per_pulse) /* fade in */
			return (((unsigned long long)vol_max) * (pulse_phase + 1) * 100) /
				(conf->pulse_rise * samples_on_per_pulse);
		else /* main run */
			return vol_max;
	} else {
		if ((pulse_phase - samples_on_per_pulse + 1) * 100 <
				conf->pulse_fall * samples_on_per_pulse) /* fade out */
			return vol_max - ((((unsigned long long)vol_max) *
				(pulse_phase - samples_on_per_pulse + 1) * 100) /
				(conf->pulse_rise * samples_on_per_pulse));

		else /* silent period */
			return 0;
	}
}

int vol_max = 0;
void audiovario_synthesise(uint16_t *buffer, size_t frames) {
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
			sample = vario_last + ((vol_max * vario_dir * freq) / SAMPLE_RATE);
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
//	print_buffer(buffer, frames);
}

