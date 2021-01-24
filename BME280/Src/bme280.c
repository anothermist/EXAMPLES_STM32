#include "bme280.h"

extern I2C_HandleTypeDef BME280_I2C;

    uint16_t    dig_T1;
    int16_t     dig_T2, dig_T3;
    uint16_t    dig_P1;
    int16_t     dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint16_t    dig_H1, dig_H3;
    int16_t     dig_H2, dig_H4, dig_H5, dig_H6;
    int32_t     t_fine;
 
void BME280_Init(void)
{
    uint8_t cmd[18];

    cmd[0] = 0xF2; // ctrl_hum
    cmd[1] = 0x05; // Humidity oversampling x16
		HAL_I2C_Master_Transmit(&BME280_I2C, BME280_ADDR, cmd, 2, BME280_I2C_TIMEOUT);
	
    cmd[0] = 0xF4; // ctrl_meas
    cmd[1] = 0xB7; // Temparature oversampling x16, Pressure oversampling x16, Normal mode
		HAL_I2C_Master_Transmit(&BME280_I2C, BME280_ADDR, cmd, 2, BME280_I2C_TIMEOUT);

    cmd[0] = 0xF5; // config
    cmd[1] = 0xa0; // Standby BME280_I2C_TIMEOUTms, Filter off
		HAL_I2C_Master_Transmit(&BME280_I2C, BME280_ADDR, cmd, 2, BME280_I2C_TIMEOUT);

    cmd[0] = 0x88; // read dig_T regs
		HAL_I2C_Master_Transmit(&BME280_I2C, BME280_ADDR, cmd, 1, BME280_I2C_TIMEOUT);
		HAL_I2C_Master_Receive(&BME280_I2C, BME280_ADDR, cmd, 6, BME280_I2C_TIMEOUT); 

    dig_T1 = (cmd[1] << 8) | cmd[0];
    dig_T2 = (cmd[3] << 8) | cmd[2];
    dig_T3 = (cmd[5] << 8) | cmd[4];

    cmd[0] = 0x8E; // read dig_P regs
		HAL_I2C_Master_Transmit(&BME280_I2C, BME280_ADDR, cmd, 1, BME280_I2C_TIMEOUT);
		HAL_I2C_Master_Receive(&BME280_I2C, BME280_ADDR, cmd, 18, BME280_I2C_TIMEOUT); 
 
    dig_P1 = (cmd[ 1] << 8) | cmd[ 0];
    dig_P2 = (cmd[ 3] << 8) | cmd[ 2];
    dig_P3 = (cmd[ 5] << 8) | cmd[ 4];
    dig_P4 = (cmd[ 7] << 8) | cmd[ 6];
    dig_P5 = (cmd[ 9] << 8) | cmd[ 8];
    dig_P6 = (cmd[11] << 8) | cmd[10];
    dig_P7 = (cmd[13] << 8) | cmd[12];
    dig_P8 = (cmd[15] << 8) | cmd[14];
    dig_P9 = (cmd[17] << 8) | cmd[16];
 
    cmd[0] = 0xA1; // read dig_H regs
		HAL_I2C_Master_Transmit(&BME280_I2C, BME280_ADDR, cmd, 1, BME280_I2C_TIMEOUT);
		HAL_I2C_Master_Receive(&BME280_I2C, BME280_ADDR, cmd, 1, BME280_I2C_TIMEOUT); 

     cmd[1] = 0xE1; // read dig_H regs

		HAL_I2C_Master_Transmit(&BME280_I2C, BME280_ADDR, &cmd[1], 1, BME280_I2C_TIMEOUT);
		HAL_I2C_Master_Receive(&BME280_I2C, BME280_ADDR, &cmd[1], 7, BME280_I2C_TIMEOUT); 

    dig_H1 = cmd[0];
    dig_H2 = (cmd[2] << 8) | cmd[1];
    dig_H3 = cmd[3];
    dig_H4 = (cmd[4] << 4) | (cmd[5] & 0x0f);
    dig_H5 = (cmd[6] << 4) | ((cmd[5]>>4) & 0x0f);
    dig_H6 = cmd[7];
}
 
float BME280_getTemperature(void)
{
    uint32_t temp_raw;
    float tempf;
    uint8_t cmd[4];
 
    cmd[0] = 0xFA; // temp_msb
		HAL_I2C_Master_Transmit(&BME280_I2C, BME280_ADDR, cmd, 1, BME280_I2C_TIMEOUT);
		HAL_I2C_Master_Receive(&BME280_I2C, BME280_ADDR, &cmd[1], 3, BME280_I2C_TIMEOUT);
 
    temp_raw = (cmd[1] << 12) | (cmd[2] << 4) | (cmd[3] >> 4);
 
    int32_t temp;
 
    temp =
        (((((temp_raw >> 3) - (dig_T1 << 1))) * dig_T2) >> 11) +
        ((((((temp_raw >> 4) - dig_T1) * ((temp_raw >> 4) - dig_T1)) >> 12) * dig_T3) >> 14);
 
    t_fine = temp;
    temp = (temp * 5 + 128) >> 8;
    tempf = (float)temp;
 
    return (tempf/100.0f);
}
 
float BME280_getHumidity(void)
{
    uint32_t hum_raw;
    float humf;
    uint8_t cmd[4];
 
    cmd[0] = 0xFD; // hum_msb
		HAL_I2C_Master_Transmit(&BME280_I2C, BME280_ADDR, cmd, 1, BME280_I2C_TIMEOUT);
		HAL_I2C_Master_Receive(&BME280_I2C, BME280_ADDR, &cmd[1], 2, BME280_I2C_TIMEOUT);
 
    hum_raw = (cmd[1] << 8) | cmd[2];
 
    int32_t v_x1;
 
    v_x1 = t_fine - 76800;
    v_x1 =  (((((hum_raw << 14) -(((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1)) +
               ((int32_t)16384)) >> 15) * (((((((v_x1 * (int32_t)dig_H6) >> 10) *
                                            (((v_x1 * ((int32_t)dig_H3)) >> 11) + 32768)) >> 10) + 2097152) *
                                            (int32_t)dig_H2 + 8192) >> 14));
    v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * (int32_t)dig_H1) >> 4));
    v_x1 = (v_x1 < 0 ? 0 : v_x1);
    v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
 
    humf = (float)(v_x1 >> 12);
 
    return (humf/1024.0f);
}

uint16_t BME280_getPressure(void)
{
    uint32_t press_raw;
    float pressf;
    uint8_t cmd[4];
 
    cmd[0] = 0xF7; // press_msb
		HAL_I2C_Master_Transmit(&BME280_I2C, BME280_ADDR, cmd, 1, BME280_I2C_TIMEOUT);
		HAL_I2C_Master_Receive(&BME280_I2C, BME280_ADDR, &cmd[1], 3, BME280_I2C_TIMEOUT);
 
    press_raw = (cmd[1] << 12) | (cmd[2] << 4) | (cmd[3] >> 4);
 
    int32_t var1, var2;
    uint32_t press;
 
    var1 = (t_fine >> 1) - 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * dig_P6;
    var2 = var2 + ((var1 * dig_P5) << 1);
    var2 = (var2 >> 2) + (dig_P4 << 16);
    var1 = (((dig_P3 * (((var1 >> 2)*(var1 >> 2)) >> 13)) >> 3) + ((dig_P2 * var1) >> 1)) >> 18;
    var1 = ((32768 + var1) * dig_P1) >> 15;
    if (var1 == 0) {
        return 0;
    }
    press = (((1048576 - press_raw) - (var2 >> 12))) * 3125;
    if(press < 0x80000000) {
        press = (press << 1) / var1;
    } else {
        press = (press / var1) * 2;
    }
    var1 = ((int32_t)dig_P9 * ((int32_t)(((press >> 3) * (press >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(press >> 2)) * (int32_t)dig_P8) >> 13;
    press = (press + ((var1 + var2 + dig_P7) >> 4));
 
    pressf = (float)press;
    return (pressf/100.0f);
}

