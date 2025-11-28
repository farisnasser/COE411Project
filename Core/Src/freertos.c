/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : freertos.c
  * @brief          : Smart Door System – Strict Order 2FA (RFID → PIN)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "rfid.h"
#include "keypad.h"
#include "tim.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* ============================= PRINT MACRO ============================= */
#define UART_PRINT(msg)                                        \
    do {                                                       \
        taskENTER_CRITICAL();                                  \
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY); \
        taskEXIT_CRITICAL();                                   \
    } while(0)
// Declare the send_event function
void send_event(char* event_message);


/* ============================= DEFINES ============================= */
#define DEFAULT_PIN      "2580"
#define ADMIN_PIN_STR    "1705"

#define EV_RFID_OK   (1u << 0)
#define EV_PIN_OK    (1u << 1)

#define UNLOCK_TIME_MS   5000U
#define RFID_SCANNED "RFID_SCANNED"
#define PIN_CORRECT "PIN_CORRECT"
#define PIN_WRONG "PIN_WRONG"
#define ACCESS_GRANTED "ACCESS_GRANTED"
#define ACCESS_DENIED "ACCESS_DENIED"
#define RFID_BAD "RFID_BAD"  // Define the RFID_BAD event


/* ============================= VARIABLES ============================= */
osEventFlagsId_t evAuth;

volatile uint8_t rfid_passed     = 0;
volatile uint8_t wrong_attempts  = 0;
volatile uint8_t lockout_active  = 0;

volatile uint8_t admin_mode      = 0;
volatile uint8_t admin_step      = 0;

/* PIN storage */
char system_pin[5]   = DEFAULT_PIN;
const char admin_pin[5] = ADMIN_PIN_STR;

char new_pin[5]      = {0};
char confirm_pin[5]  = {0};

/* Task prototypes */
void StartAccessTask(void *argument);
void StartRfidTask(void *argument);
void StartKeypadTask(void *argument);

/* RFID prototypes */
uint8_t RFID_CheckTag(char *tagOut);
void    RFID_Init_IT(void);

/* From main.c */
void Door_Lock(void);
void Door_Unlock(void);
void chime_ok(void);
void chime_error(void);

/* ============================= INIT ============================= */
void MX_FREERTOS_Init(void)
{
    evAuth = osEventFlagsNew(NULL);

    osThreadNew(StartAccessTask, NULL,
        &(osThreadAttr_t){ .name="ACCESS", .stack_size=1024, .priority=osPriorityHigh });

    osThreadNew(StartRfidTask, NULL,
        &(osThreadAttr_t){ .name="RFID", .stack_size=768, .priority=osPriorityNormal });

    osThreadNew(StartKeypadTask, NULL,
        &(osThreadAttr_t){ .name="KEYPAD", .stack_size=768, .priority=osPriorityNormal });
}

/* =====================================================================
 * ACCESS TASK – Waits for RFID first, then PIN
 * ===================================================================== */
void StartAccessTask(void *argument)
{
    Door_Lock();
    UART_PRINT("\r\n[ACCESS] System Ready. Please scan RFID card...\r\n");

    for (;;)
    {
        osEventFlagsWait(evAuth,
                         EV_RFID_OK | EV_PIN_OK,
                         osFlagsWaitAll,
                         osWaitForever);

        UART_PRINT("[ACCESS] ★ Authentication Complete → UNLOCKING DOOR ★\r\n");

        /* ================= MQTT EVENT ================= */
        UART_PRINT("[MQTT] EVENT:ACCESS_GRANTED\r\n");

        Door_Unlock();
        chime_ok();

        /* Reset state */
        rfid_passed    = 0;
        wrong_attempts = 0;
        osEventFlagsClear(evAuth, EV_RFID_OK | EV_PIN_OK);

        osDelay(UNLOCK_TIME_MS);

        Door_Lock();
        UART_PRINT("[ACCESS] Door Locked. Scan RFID to start again.\r\n");
    }
}

/* =====================================================================
 * RFID TASK
 * ===================================================================== */
void StartRfidTask(void *argument)
{
    char tag[20];
    RFID_Init_IT();
    UART_PRINT("[RFID] Ready (Interrupt Mode)\r\n");

    for (;;)
    {
        if (lockout_active)
        {
            osDelay(100);
            continue;
        }

        uint8_t result = RFID_CheckTag(tag);	//0 if not ready, 1 if correct, 2 wrong

        if (result == 1)  // Authorized RFID card
        {
            if (!rfid_passed)
            {
                UART_PRINT("[RFID] Authorized Card → RFID OK\r\n");

                // ================= MQTT EVENT =================
                send_event(RFID_SCANNED);  // Send the RFID scanned event to ESP12F

                UART_PRINT("[RFID] Now enter your PIN...\r\n");

                rfid_passed = 1;
                osEventFlagsSet(evAuth, EV_RFID_OK);
            }
        }
        else if (result == 2)  // Unauthorized RFID card
        {
            UART_PRINT("[RFID] Unauthorized Card\r\n");

            // ================= MQTT EVENT =================
            send_event(RFID_BAD);  // Send the unauthorized RFID event to ESP12F

            chime_error();
        }

        osDelay(50);
    }
}


/* =====================================================================
 * KEYPAD TASK – PIN + Admin Mode
 * ===================================================================== */
void StartKeypadTask(void *argument)
{
    Keypad_Init();
    char pin[5] = {0};
    uint8_t idx = 0;

    for (;;)
    {
        /* ================= LOCKOUT ================= */
        if (lockout_active)
        {
            UART_PRINT("[SYSTEM] LOCKED OUT — WAITING 10 SECONDS...\r\n");
            osDelay(10000);

            UART_PRINT("[SYSTEM] Lockout cleared. Scan RFID again.\r\n");

            lockout_active = 0;
            wrong_attempts = 0;
            rfid_passed    = 0;
            admin_mode     = 0;
            admin_step     = 0;

            idx = 0;
            memset(pin, 0, sizeof(pin));
            memset(new_pin, 0, sizeof(new_pin));
            memset(confirm_pin, 0, sizeof(confirm_pin));

            osEventFlagsClear(evAuth, EV_RFID_OK | EV_PIN_OK);
            continue;
        }

        /* ================= KEYPAD SCAN ================= */
        char k = Keypad_Scan();

        if (!k)
        {
            osDelay(80);
            continue;
        }

        char msg[40];
        sprintf(msg, "[KEYPAD] Key Pressed: %c\r\n", k);
        UART_PRINT(msg);

        /* ================= ADMIN MODE ================= */
        if (admin_mode)
        {
            // unchanged logic
            osDelay(80);
            continue;
        }

        /* ================= NORMAL MODE ================= */

        if (!rfid_passed)
        {
            UART_PRINT("[KEYPAD] PIN ignored. Scan RFID first.\r\n");
            osDelay(80);
            continue;
        }

        if (k == '#')
        {
            idx = 0;
            memset(pin, 0, sizeof(pin));
            UART_PRINT("[KEYPAD] PIN Cleared\r\n");
        }
        else if (k == '*')
        {
            if (idx != 4)
            {
                UART_PRINT("[KEYPAD] PIN must be 4 digits\r\n");
                idx = 0;
                memset(pin, 0, sizeof(pin));
                osDelay(80);
                continue;
            }

            if (memcmp(pin, system_pin, 4) == 0)
            {
                UART_PRINT("[KEYPAD] ✔ Correct PIN → PIN OK\r\n");

                // ================= MQTT EVENT =================
                send_event(PIN_CORRECT);  // Send the PIN correct event to ESP12F

                osEventFlagsSet(evAuth, EV_PIN_OK);
                wrong_attempts = 0;
            }
            else
            {
                wrong_attempts++;

                char err[80];
                sprintf(err, "[KEYPAD] ✖ Incorrect PIN (%d/3)\r\n", wrong_attempts);
                UART_PRINT(err);

                // ================= MQTT EVENT =================
                send_event(PIN_WRONG);  // Send the PIN wrong event to ESP12F

                chime_error();

                if (wrong_attempts >= 3)
                {
                    UART_PRINT("\r\n[SYSTEM] TOO MANY WRONG ATTEMPTS — LOCKING FOR 10 SECONDS\r\n\n");
                    lockout_active = 1;

                    // ================= MQTT EVENT =================
                    send_event(ACCESS_DENIED);  // Send access denied event to ESP12F
                }
                else
                {
                    UART_PRINT("[KEYPAD] Scan RFID again.\r\n");
                    rfid_passed = 0;
                    osEventFlagsClear(evAuth, EV_RFID_OK);
                }
            }

            idx = 0;
            memset(pin, 0, sizeof(pin));
        }
        else if (k >= '0' && k <= '9')
        {
            if (idx < 4)
            {
                pin[idx++] = k;
            }
            else
            {
                UART_PRINT("[KEYPAD] Max 4 digits. Press * or #.\r\n");
            }
        }

        osDelay(80);
    }
}

