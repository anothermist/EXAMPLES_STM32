//  -------------------------------
// | 1 VDD | 2 SDA | 3 GND | 4 SCL |  NEED PULL UP RESISTORS !!!
//  -------------------------------

#ifndef AM2320_H_
#define AM2320_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define AM2320_I2C hi2c1
#define AM2320_ADDR ((0x5C) << 1)  // I2C 7-bit address shifted for HAL
#define AM2320_I2C_TIMEOUT 100
#define HT_CMD_MSB  0x03  // Read register command
#define HT_CMD_LSB  0x00  // Starting at register 0x00 (humidity + temp)

void AM2320_Update(void);
float AM2320_getTemperature(void);
float AM2320_getHumidity(void);

#ifdef __cplusplus
}
#endif

#endif /* AM2320_H_ */
