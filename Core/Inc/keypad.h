#ifndef __KEYPAD_H__
#define __KEYPAD_H__

#include "stm32l4xx_hal.h"

/* Adjust debounce time (ms) */
#define DEBOUNCE_MS 20

/* Public function prototypes */
void Keypad_Init(void);
char Keypad_Scan(void);

#endif /* __KEYPAD_H__ */
