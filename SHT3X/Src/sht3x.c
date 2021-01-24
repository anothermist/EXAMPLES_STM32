#include "sht3x.h"

extern I2C_HandleTypeDef SHT3X_I2C;

static uint8_t measure_data[6];

void SHT3X_Update(void) 
{
  uint8_t cmd[2];
  cmd[0] = HT_CMD_MSB;
  cmd[1] = HT_CMD_LSB;

	HAL_I2C_Master_Transmit(&SHT3X_I2C, SHT3X_ADDR, cmd, 2, SHT3X_I2C_TIMEOUT);
	HAL_I2C_Master_Receive(&SHT3X_I2C, SHT3X_ADDR, measure_data, 6, SHT3X_I2C_TIMEOUT); 
}

float SHT3X_getTemperature(void)
{
	float temperatureC;
	uint16_t temperature_raw;
	temperature_raw = measure_data[0];
  temperature_raw = temperature_raw << (uint16_t) 8 ;
  temperature_raw = temperature_raw + (uint16_t) measure_data[1];
  temperatureC = 175.0 * (float) temperature_raw / 65535.0;
  temperatureC = temperatureC - 45;
	return temperatureC;
}

float SHT3X_getHumidity(void)
{
	float humidity;
	uint16_t humidity_raw;
  humidity_raw = measure_data[3];
  humidity_raw = humidity_raw << (uint16_t) 8;
  humidity_raw = humidity_raw + (uint16_t) measure_data[4];
  humidity = 100.0 * (float) humidity_raw / 65535.0;
	return humidity;
}
