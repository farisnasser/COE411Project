#include "rfid.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

#define RFID_START    0x0A     // Start-of-frame character (LF)
#define RFID_END      0x0D     // End-of-frame character (CR)
#define RFID_BUF_LEN  32       // Max length for received tag

#define AUTH_TAG "5200129EA6"  // Authorized RFID tag ID

/* Internal UART receive state */
static uint8_t  rfid_rx_byte;             // Holds each received byte
static char     rfid_buffer[RFID_BUF_LEN];// Buffer for the full tag
static uint8_t  rfid_index = 0;           // Current write index
static volatile uint8_t rfid_tag_ready = 0; // Set when a complete tag is received

/* -------------------------------------------------------------------------
 * Initialize RFID UART (USART1) to receive bytes using interrupts.
 * ------------------------------------------------------------------------- */
void RFID_Init_IT(void)
{
    rfid_index      = 0;
    rfid_buffer[0]  = '\0';
    rfid_tag_ready  = 0;

    // Start first interrupt-based receive on USART1 (1 byte)
    HAL_UART_Receive_IT(&huart1, &rfid_rx_byte, 1);
}

/* -------------------------------------------------------------------------
 * UART RX callback: runs every time one byte arrives from the RFID module.
 * Builds the tag between start (0x0A) and end (0x0D) characters.
 * ------------------------------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
        return;   // Ignore other UARTs

    uint8_t b = rfid_rx_byte;

    if (b == RFID_START)
    {
        // Start a new frame
        rfid_index     = 0;
        rfid_tag_ready = 0;
    }
    else if (b == RFID_END)
    {
        // End of frame → terminate string
        if (rfid_index < RFID_BUF_LEN)
            rfid_buffer[rfid_index] = '\0';
        else
            rfid_buffer[RFID_BUF_LEN - 1] = '\0';

        rfid_tag_ready = 1;   // Notify main code a full tag is ready
    }
    else
    {
        // Store tag characters while avoiding overflow
        if (rfid_index < RFID_BUF_LEN - 1)
            rfid_buffer[rfid_index++] = b;
    }

    // Re-enable interrupt to receive next byte
    HAL_UART_Receive_IT(&huart1, &rfid_rx_byte, 1);
}

/* -------------------------------------------------------------------------
 * Check if a complete tag has been received.
 * return 0 → no tag
 *        1 → authorized (matches AUTH_TAG)
 *        2 → unauthorized
 * ------------------------------------------------------------------------- */
uint8_t RFID_CheckTag(char *tagOut)
{
    if (!rfid_tag_ready)
        return 0;  // No complete tag received yet

    // Copy tag to user buffer
    strcpy(tagOut, rfid_buffer);

    // Reset for next read
    rfid_tag_ready = 0;
    rfid_index     = 0;
    rfid_buffer[0] = '\0';

    // Compare against authorized tag
    if (strcmp(tagOut, AUTH_TAG) == 0)
        return 1;
    else
        return 2;
}

/* -------------------------------------------------------------------------
 * Legacy placeholder functions (kept to avoid linker errors).
 * ------------------------------------------------------------------------- */
uint8_t RFID_ReadTag(UART_HandleTypeDef *rfid_uart,
                     UART_HandleTypeDef *debug_uart,
                     char *tagBuffer)
{
    (void)rfid_uart;
    (void)debug_uart;
    (void)tagBuffer;
    return 0;
}

void RFID_Init(UART_HandleTypeDef *rfid_uart, UART_HandleTypeDef *debug_uart)
{
    (void)rfid_uart;
    (void)debug_uart;
}
