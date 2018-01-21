/* Lark TE vario computer
 * Copyright (C) 2018 Tomas Hlavacek (tomas.hlavacek@akaflieg.tu-darmstadt.de)
 * Copyright (C) 2014  The openvario project
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
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "driver/i2c.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "sdkconfig.h"

#include "ms5611.h"
#include "net.h"
#include "audiovario.h"


#define STACK_SIZE 4096
#define TAG "lark-vario: "


#define I2C_SCL 22
#define I2C_SDA 21
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0
#define I2C_MASTER_FREQ_HZ         100000
#define I2C_TEP_ADDR 0x77

#define CONVERSION_BEAT_US 12500


float vario_val;
float dynamic_pressure;
float static_pressure;


/* TE vario computation from Kalman filter output. Taken from OpenVario. */
static float ComputeVario(const float p, const float d_p)
{
  static const float FACTOR = -2260.389548275485;
  static const float EXP = -0.8097374740609689;
  return FACTOR*pow(p, EXP) * d_p;
}


/* Kalman filter implementation taken from OpenVario. */
typedef struct {
        float x_abs_;           // the absolute quantity x
        float x_vel_;           // the rate of change of x, in x units per second squared.
        
        // Covariance matrix for the state
        float p_abs_abs_;
        float p_abs_vel_;
        float p_vel_vel_;
        
        // The variance of the acceleration noise input to the system model
        float var_x_accel_;
} t_kalmanfilter1d;

t_kalmanfilter1d vkf;

static void KalmanFiler1d_update(t_kalmanfilter1d* filter, float z_abs, float var_z_abs, float dt)
{
	float F1=1.0;
	float dt2, dt3, dt4;
	float y;
	float s_inv;
	float k_abs;
	float k_vel;
	
	// check if dt is positive
	
	//Predict step
	//update state estimate
	filter->x_abs_ += filter->x_vel_ * dt;
	
	// update state covariance
	dt2 = dt * dt;
	dt3 = dt * dt2;
	dt4 = dt2 * dt2;

	filter->p_abs_abs_ += 2*(dt*filter->p_abs_vel_) + dt2 * filter->p_vel_vel_ + 0.25*(filter->var_x_accel_ * dt4);
	filter->p_abs_vel_ += dt * filter->p_vel_vel_ + (filter->var_x_accel_ * dt3)/2;
	filter->p_vel_vel_ += filter->var_x_accel_ * dt2;
	
	// Update step
	y = z_abs - filter->x_abs_;		// Innovation
	s_inv = F1 / (filter->p_abs_abs_ + var_z_abs);		// Innovation precision 
	k_abs = filter->p_abs_abs_*s_inv; // Kalman gain
	k_vel = filter->p_abs_vel_*s_inv;
	
	// Update state estimate.
	filter->x_abs_ += k_abs * y;
	filter->x_vel_ += k_vel * y;
  
	// Update state covariance.
	filter->p_vel_vel_ -= filter->p_abs_vel_ * k_vel;
	filter->p_abs_vel_ -= filter->p_abs_vel_ * k_abs;
	filter->p_abs_abs_ -= filter->p_abs_abs_ * k_abs;
}

static void KalmanFilter1d_reset(t_kalmanfilter1d* filter)
{
	filter->x_abs_ = 0.0;
	filter->x_vel_ = 0.0;

	filter->p_abs_abs_ = 0.0;
	filter->p_abs_vel_ = 0.0;
	filter->p_vel_vel_ = 0.0;
	
	filter->var_x_accel_ = 0.0;
}



void sensor_read_round(ms5611_drv_t *tep_dev, ms5611_drv_t *dp_dev, ms5611_drv_t *sp_dev) {
	static int stage = 1;

	static int32_t rawpress = 0;
	static int32_t rawtemp = 0;
	float tep_read;
	float dp_read;
	float sp_read;
	float temp;

	switch (stage) {
		case 1:
                case 5:
                case 9:
                case 13:
                case 17:
                case 21:
                case 25:
                case 29:
                case 33:
                case 37:
			ms5611_start_conv_press(tep_dev);
			// TODO: start conv sp_dev and dp_dev
			break;
		case 2:
                case 6:
                case 10:
                case 14:
                case 18:
                case 22:
                case 26:
                case 30:
                case 34:
                case 38:
			/* TEK pressure */
			rawpress = ms5611_get_conv(tep_dev);
			tep_read = ms5611_get_pressure(tep_dev, rawpress, rawtemp);
			ESP_LOGD(TAG, "read tep: %f (%fm)\n", tep_read, ms5611_calc_altitude(tep_read));

			// check tep_pressure input value for validity
			if ((tep_read < 100) || (tep_read > 1200)) {
				/* TEK pressure out of range */
				ESP_LOGW(TAG, "%s warn tep out of range\n", __func__);
			} else {
				KalmanFiler1d_update(&vkf, tep_read, 0.25, 0.05);
			}

			/* Total pressure + low-pass */
			// TODO: Read
			dp_read = 0;
			dynamic_pressure = (3*dynamic_pressure + dp_read)/4;

			/* mask speeds < 10km/h */
			if (dynamic_pressure < 0.04) {
				dynamic_pressure = 0.0;
			}

			/* Static pressure + low-pass */
			// TODO: Read
			sp_read = 0;
			static_pressure = (3*static_pressure + sp_read)/4;
			break;
		case 3:
                        /* Start temp measurement */
                        ms5611_start_conv_temp(tep_dev);
                        break;
                case 4:
                        /* read temp values */
			rawtemp = ms5611_get_conv(tep_dev);
			temp = ms5611_get_temp(tep_dev, rawtemp);
			ESP_LOGD(TAG, "%s temp=%f\n", __func__, temp);
                        break;
                default:
                        break;
	}

	if (stage >= 40)
		stage = 1;
	else
		stage++;
}

void sensor_update_output(void) {
	static int stage = 1;
	switch (stage) {
		case 5:
                case 10:
                case 15:
                case 20:
                case 25:
                case 30:
                case 35:
                case 40:
			/* compute current TE vario value */
			vario_val = ComputeVario(vkf.x_abs_, vkf.x_vel_);
			ESP_LOGD(TAG, "%s: vario_val=%f\n", __func__, vario_val);
			/* Unblock network -> feed data */
			xSemaphoreGive(net_feed_semaphore);
			/* Unblock audio update -> feed data */
			xSemaphoreGive(audio_feed_semaphore);
			break;
		default:
			break;
	}

	if (stage >= 40)
		stage = 1;
	else
		stage++;
}

static float compute_pressure(ms5611_drv_t *dev) {
	int32_t rawtemp;
	int32_t rawpress;

	ms5611_start_conv_temp(dev);
	vTaskDelay(2*CONVERSION_BEAT_US/(1000*portTICK_PERIOD_MS));
	rawtemp = ms5611_get_conv(dev);

	ms5611_start_conv_press(dev);
	vTaskDelay(2*CONVERSION_BEAT_US/(1000*portTICK_PERIOD_MS));
	rawpress = ms5611_get_conv(dev);

	return ms5611_get_pressure(dev, rawpress, rawtemp);
}

/* Sensor device structs */
ms5611_drv_t tep_dev;
ms5611_drv_t dp_dev;
ms5611_drv_t sp_dev;
SemaphoreHandle_t timer_semaphore = NULL;

static void sensor_read_timer_callback(void *arg) {
	xSemaphoreGive(timer_semaphore);
}

static void sensor_read_task(void *pvParameter) {
	/* run main loop */
	while(1) {
		if (xSemaphoreTake(timer_semaphore, portMAX_DELAY)!= pdTRUE)
			ESP_LOGW(TAG, "semaphore failed!\n");
		sensor_read_round(&tep_dev, &dp_dev, &sp_dev);
		sensor_update_output();
	}
}


int sensor_read_init(void) {
	/* I2C for sensors */
	int i2c_master_port = I2C_NUM_0;
	i2c_config_t i2s_conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_SDA,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = I2C_SCL,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master = {
			.clk_speed = I2C_MASTER_FREQ_HZ
		}
	};

	i2c_param_config(i2c_master_port, &i2s_conf);
	i2c_driver_install(i2c_master_port, i2s_conf.mode,
		I2C_MASTER_RX_BUF_DISABLE,
		I2C_MASTER_TX_BUF_DISABLE, 0);

	/* Kalman filter */
	KalmanFilter1d_reset(&vkf);
	vkf.var_x_accel_ = 0.3; /* taken from openvario sensorsd.conf */
 
	int ret = ms5611_init(&tep_dev, I2C_NUM_0, I2C_TEP_ADDR);
	ESP_LOGD(TAG, "%s: ms5611 init: %d\n", __func__, ret);

	float tep_init = 0;
	int sensor_test = 0;
	while ((tep_init < 100) || (tep_init > 1200) || (sensor_test > 1000)) {
		tep_init = compute_pressure(&tep_dev);
		sensor_test++;
	}
	if (sensor_test >= 1000) {
		ESP_LOGE(TAG, "Can not read from TEP sensor. Finish.");
		return ESP_FAIL;
	}

        for(int i=0; i < 1000; i++)
                KalmanFiler1d_update(&vkf, tep_init, 0.25, 1);

	/* create read semaphore */
	timer_semaphore = xSemaphoreCreateBinary();;

	/* create read task */
	xTaskCreate(&sensor_read_task, "sensor_read_task", STACK_SIZE, NULL, 6, NULL);

	esp_timer_handle_t read_timer;
	esp_timer_create_args_t timer_conf = {
		.callback = &sensor_read_timer_callback,
		.dispatch_method = ESP_TIMER_TASK
	};
	esp_timer_create(&timer_conf, &read_timer);
	esp_timer_start_periodic(read_timer, CONVERSION_BEAT_US);

	return ESP_OK;
}


