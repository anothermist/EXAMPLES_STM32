#include "stm32f1xx_hal.h"
#include <stdio.h>
#include "main.h"

extern TIM_HandleTypeDef htim4;

#define RECIV_PIN (HAL_GPIO_ReadPin(IR_Input_GPIO_Port, IR_Input_Pin))

#define RAWBUF 256

#define STATE_IDLE      2
#define STATE_MARK      3
#define STATE_SPACE     4
#define STATE_STOP      5
#define STATE_OVERFLOW  6

#define GAP_TICKS       100

// IR detector output is active low
#define MARK   0
#define SPACE  1

typedef struct // The fields are ordered to reduce memory over caused by struct-padding
{ 
	uint8_t       rcvstate;        // State Machine state
	uint8_t       rawlen;          // counter of entries in rawbuf
	uint16_t      timer;           // State timer, counts 50uS ticks.
	uint16_t      rawbuf[RAWBUF];  // raw data
	uint8_t       overflow;        // Raw buffer overflow occurred
} irparams_t;

typedef struct 
{
		uint32_t value; // Decoded value [max 32-bits]
		int16_t bits; // Number of bits in decoded value
		volatile uint16_t *rawbuf; // Raw intervals in 50uS ticks
		int16_t rawlen; // Number of records in rawbuf
		int16_t overflow; // true iff IR raw code too long
} decode_results;

int16_t my_decode(decode_results *results);
int32_t decodeHash(decode_results *results);
void my_enableIRIn();
void my_resume();
