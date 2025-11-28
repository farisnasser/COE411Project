#include "keypad.h"

/* === GPIO pin configuration (adjust as needed) === */
/* Rows are driven as outputs, columns are read as inputs */
#define KEYPAD_PORT GPIOC
static const uint16_t ROW_PINS[4] = { GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3 };
static const uint16_t COL_PINS[4] = { GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7 };
/* ================================================= */

/* Key layout (4x4) */
static const char KEYMAP[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

/* Initialize keypad (set rows low at start) */
void Keypad_Init(void)
{
    HAL_GPIO_WritePin(KEYPAD_PORT,
                      ROW_PINS[0] | ROW_PINS[1] | ROW_PINS[2] | ROW_PINS[3],
                      GPIO_PIN_RESET);
}

/* Drive all rows low */
static void Rows_AllLow(void)
{
    HAL_GPIO_WritePin(KEYPAD_PORT,
                      ROW_PINS[0] | ROW_PINS[1] | ROW_PINS[2] | ROW_PINS[3],
                      GPIO_PIN_RESET);
}

/* Scan keypad: returns key pressed, or 0 if none */
char Keypad_Scan(void)
{
    Rows_AllLow();

    for (int r = 0; r < 4; r++)
    {
        HAL_GPIO_WritePin(KEYPAD_PORT, ROW_PINS[r], GPIO_PIN_SET);

        for (int c = 0; c < 4; c++)
        {
            if (HAL_GPIO_ReadPin(KEYPAD_PORT, COL_PINS[c]) == GPIO_PIN_SET)
            {
                HAL_Delay(DEBOUNCE_MS);//debounce here since it might spike high multiple times

                if (HAL_GPIO_ReadPin(KEYPAD_PORT, COL_PINS[c]) == GPIO_PIN_SET)//same if statement to check again, if its high, then in the if statement will actually write pin
                {
                    /* Wait for release */
                    while (HAL_GPIO_ReadPin(KEYPAD_PORT, COL_PINS[c]) == GPIO_PIN_SET);

                    HAL_GPIO_WritePin(KEYPAD_PORT, ROW_PINS[r], GPIO_PIN_RESET);
                    return KEYMAP[r][c];
                }
            }
        }
        HAL_GPIO_WritePin(KEYPAD_PORT, ROW_PINS[r], GPIO_PIN_RESET);
    }

    return 0; // No key pressed
}
