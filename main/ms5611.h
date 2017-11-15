#ifndef MS5611_H
#define MS5611_H

//#include <stdbool.h>

// addresses of the device
#define MS5611_ADDR_CSB_HIGH  0x76   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS5611_ADDR_CSB_LOW   0x77   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)

// registers of the device
#define MS5611_D1 0x40
#define MS5611_D2 0x50
#define MS5611_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS5611_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS5611_OSR_256 0x00
#define MS5611_OSR_512 0x02
#define MS5611_OSR_1024 0x04
#define MS5611_OSR_2048 0x06
#define MS5611_OSR_4096 0x08
#define MS5611_OSR_DEFAULT MS5611_OSR_4096

#define MS5611_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values.
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS5611_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS5611_PROM_REG_SIZE 2 // size in bytes of a prom registry.

// Self test parameters. Only checks that values are sane
#define MS5611_ST_PRESS_MAX   (1100.0) //mbar
#define MS5611_ST_PRESS_MIN   (450.0)  //mbar
#define MS5611_ST_TEMP_MAX    (60.0)   //degree celcius
#define MS5611_ST_TEMP_MIN    (-20.0)  //degree celcius

// Constants used to determine altitude from pressure
#define CONST_SEA_PRESSURE 102610.f //1026.1f //http://www.meteo.physik.uni-muenchen.de/dokuwiki/doku.php?id=wetter:stadt:messung
#define CONST_PF 0.1902630958 //(1/5.25588f) Pressure factor
#define CONST_PF2 44330.0f


/*bool ms5611Init(I2C_TypeDef *i2cPort);
bool ms5611SelfTest(void);
bool ms5611EvaluateSelfTest(float min, float max, float value, char* string);
float ms5611GetPressure(uint8_t osr);
float ms5611CalcPressure(int32_t rawPress, int32_t dT);
float ms5611GetTemperature(uint8_t osr);
float ms5611CalcTemp(int32_t deltaT);
int32_t ms5611GetDeltaTemp(uint8_t osr);
int32_t ms5611CalcDeltaTemp(int32_t rawTemp);
int32_t ms5611RawPressure(uint8_t osr);
int32_t ms5611RawTemperature(uint8_t osr);
bool ms5611ReadPROM();
void ms5611Reset();
void ms5611StartConversion(uint8_t command);
int32_t ms5611GetConversion(uint8_t command);

void ms5611GetData(float* pressure, float* temperature, float* asl);
float ms5611PressureToAltitude(float* pressure);
*/

typedef struct calibration_reg{
	uint16_t psens;
	uint16_t off;
	uint16_t tcs;
	uint16_t tco;
	uint16_t tref;
	uint16_t tsens;
} calibration_reg_t;


typedef struct ms5611_drv {
	i2c_port_t i2c_num;
	uint8_t addr;
	int initialized;
	calibration_reg_t cr;
} ms5611_drv_t;

int ms5611_read_PROM(ms5611_drv_t *dev);
void ms5611_reset(ms5611_drv_t *dev);
int ms5611_init(ms5611_drv_t *dev, i2c_port_t i2c_num, uint8_t addr);
void ms5611_start_conv(ms5611_drv_t *dev, int8_t command);
int32_t ms5611_get_conv(ms5611_drv_t *dev);
int32_t ms5611_raw_pressure(ms5611_drv_t *dev, uint8_t osr);
int32_t ms5611_raw_temp(ms5611_drv_t *dev, uint8_t osr);


int32_t ms5611_get_deltatemp(ms5611_drv_t *dev, uint8_t osr);
float ms5611_get_temp(ms5611_drv_t *dev, uint8_t osr);
float ms5611_get_pressure(ms5611_drv_t *dev, uint8_t osr);
float ms5611_calc_altitude(float* pressure/*, float* ground_pressure, float* ground_temp*/);

#endif // MS5611_H
