#ifndef AUDIOVARIO
#define AUDIOVARIO

#define SAMPLE_RATE (44100)
#define SAMPLE_BITS (16)
#define SAMPLE_MAX ((1<<SAMPLE_BITS)-1)


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

typedef enum audiovario_mode {
  vario = 0,
  stf   = 1
} audiovario_mode_t;

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

void audiovario_init(void);
void audiovario_set_mode(audiovario_mode_t mode);
int audiovario_change_volume(int delta);
void audiovario_update(float val);
void audiovario_synthesise(uint16_t *buffer, size_t frames);

#endif

