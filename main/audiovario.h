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


#ifndef AUDIOVARIO_H
#define AUDIOVARIO_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* FreeRTOS-specific and Lark-specific config */
#define AUDIO_I2S_NUM 0
#define AUDIO_BUFFER_SIZE 4096
#define AUDIO_DMA_BUFFER_SIZE 1024
#define AUDIO_DMA_BUFFER_NUM 4
#define AUDIO_TASK_PERIOD_MS 10

#define AUDIO_SAMPLE_RATE (44100)
#define AUDIO_SAMPLE_BITS (16)
#define AUDIO_SAMPLE_MAX ((1<<AUDIO_SAMPLE_BITS)-1)


/* TE Vario config */
#define DEADBAND_LOW -0.0
#define DEADBAND_HIGH 0.0 /* remain silent for DEADBAND_LOW < TE value < DEADBAND_HIGH */
#define PULSE_LENGTH  12288 /* pulse period for positive TE values, in samples */
#define PULSE_LENGTH_GAIN 1 /* pulses get shorter with higher TE values */
#define PULSE_DUTY 40 /* percent */
#define PULSE_RISE 5 /* percent - Timing for rising edge of pulse (fade-in) */
#define PULSE_FALL 5 /* pecent - Timing for falling edge of pulse (fade-out) */
#define BASE_FREQ_POS 400 /*base frequency for positive TE values in Hz*/
#define BASE_FREQ_NEG 400 /*base frequency for negative TE values in Hz*/
#define FREQ_GAIN_POS 180
#define FREQ_GAIN_NEG 0.75

/* Speed-to-fly config */
#define STF_DEADBAND_LOW -2.5
#define STF_DEADBAND_HIGH 2.5
#define STF_PULSE_LENGTH 12288
#define STF_PULSE_LENGTH_GAIN 0.2
#define STF_PULSE_DUTY 40
#define STF_PULSE_RISE 2
#define STF_PULSE_FALL 2
#define STF_BASE_FREQ_POS 400
#define STF_BASE_FREQ_NEG 400
#define STF_FREQ_GAIN_POS 30
#define STF_FREQ_GAIN_NEG 0.1


/* Definitions */
typedef enum audiovario_mode {
  vario = 0,
  stf   = 1
} audiovario_mode_t;

extern SemaphoreHandle_t audio_feed_semaphore;

/* API definitions */

BaseType_t audiovario_start(void);
void audiovario_stop(void);
void audiovario_set_mode(audiovario_mode_t mode);
int audiovario_change_volume(int delta);
int audiovario_set_volume(int vol);
void audiovario_update(float val);

#endif

