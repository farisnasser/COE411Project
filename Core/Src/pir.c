#include "pir.h"

static volatile uint8_t  pir_event = 0;       // ISR sets, main clears
static volatile uint32_t pir_count = 0;       // optional counter
static volatile uint32_t pir_last_ms = 0;     // last accepted edge time

// Ignore extra edges while PIR is holding HIGH
#define PIR_COOLDOWN_MS   1200u

void PIR_Init(void) {
  pir_event = 0;
  pir_count = 0;
  pir_last_ms = 0;
}

// ---- ISR path, identical philosophy to your Lab 5 ----
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == PIR_Pin) {
    uint32_t now = HAL_GetTick();
    if ((now - pir_last_ms) > PIR_COOLDOWN_MS) {
      pir_last_ms = now;
      pir_count++;
      pir_event = 1;     // notify main
    } else {
      // ignore bounces/retriggers in cooldown window
    }
  }
}

bool PIR_TakeEvent(void) {
  if (pir_event) {
    __disable_irq();
    pir_event = 0;
    __enable_irq();
    return true;
  }
  return false;
}

uint32_t PIR_LastTick(void) { return pir_last_ms; }


