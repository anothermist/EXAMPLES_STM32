#include "am2320.h"

extern I2C_HandleTypeDef AM2320_I2C;

static uint8_t measure_data[8]; // 8 байт: [0]=0x03, [1]=0x04, [2..5]=data, [6..7]=CRC

void AM2320_Update(void) {
	// Wake-up: send empty transmission
	HAL_I2C_Master_Transmit(&AM2320_I2C, AM2320_ADDR, NULL, 0,
			AM2320_I2C_TIMEOUT);
	HAL_Delay(2); // 1–2 ms delay

	// Send read command: Read 4 bytes from register 0x00
	uint8_t cmd[3] = { 0x03, 0x00, 0x04 };
	if (HAL_I2C_Master_Transmit(&AM2320_I2C, AM2320_ADDR, cmd, 3,
			AM2320_I2C_TIMEOUT) != HAL_OK) {
		printf("AM2320: Failed to send read command\r\n");
		return;
	}

	HAL_Delay(2); // Wait before reading response

	// Read 8 bytes
	if (HAL_I2C_Master_Receive(&AM2320_I2C, AM2320_ADDR, measure_data, 8,
			AM2320_I2C_TIMEOUT) != HAL_OK) {
		printf("AM2320: Failed to read data\r\n");
		return;
	}

	printf("AM2320: Measurement complete\r\n");
}

float AM2320_getTemperature(void) {
	uint16_t raw = (measure_data[4] << 8) | measure_data[5];

	if (raw & 0x8000) {
		raw &= 0x7FFF;
		return -((float) raw / 10.0f);
	}
	return (float) raw / 10.0f;
}

float AM2320_getHumidity(void) {
	uint16_t raw = (measure_data[2] << 8) | measure_data[3];
	return (float) raw / 10.0f;
}

