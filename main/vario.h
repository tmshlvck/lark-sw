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


typedef struct {
	float x_abs_;		// the absolute quantity x
	float x_vel_;		// the rate of change of x, in x units per second squared.
	
	// Covariance matrix for the state
	float p_abs_abs_;
	float p_abs_vel_;
	float p_vel_vel_;
	
	// The variance of the acceleration noise input to the system model
	float var_x_accel_;
} t_kalmanfilter1d;


extern float vario_val;
extern float dynamic_pressure;
extern float static_pressure;



void KalmanFiler1d_update(t_kalmanfilter1d* , float , float , float);
void KalmanFilter1d_reset(t_kalmanfilter1d*);

float ComputeVario(const float, const float);

void sensor_read_task(void *pvParameter);

