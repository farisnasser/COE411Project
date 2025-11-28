#pragma once
#include "main.h"
#include <stdbool.h>

#define PIR_GPIO_Port   GPIOB
#define PIR_Pin         GPIO_PIN_1

void PIR_Init(void);
bool PIR_TakeEvent(void);     // returns true once per motion event
uint32_t PIR_LastTick(void);  // last trigger tick (ms)
