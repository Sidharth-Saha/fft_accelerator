#pragma once
#include <stdint.h>

typedef struct PalUart PalUart;

typedef enum {
    PAL_UART_OK = 0,
    PAL_UART_ERR_OPEN = -1,
    PAL_UART_ERR_IO = -2,
    PAL_UART_ERR_TIMEOUT = -3,
    PAL_UART_ERR_INVALID = -4
} PalUartResult;

typedef struct {
    const char* device;      // e.g. "COM3" or "/dev/ttyUSB0"
    uint32_t    baudrate;    // e.g. 115200
    uint8_t     databits;    // usually 8
    char        parity;      // 'N', 'E', 'O'
    uint8_t     stopbits;    // 1 or 2
    uint32_t    timeout_ms;  // read timeout
} PalUartConfig;

// Open and configure UART
PalUartResult pal_uart_open(PalUart** out_uart, const PalUartConfig* cfg);

// Close UART
void pal_uart_close(PalUart* uart);

// Blocking write: tries to write len bytes
PalUartResult pal_uart_write(PalUart* uart,
    const uint8_t* data,
    size_t len,
    size_t* bytes_written);

// Blocking read with timeout from config
PalUartResult pal_uart_read(PalUart* uart,
    uint8_t* buf,
    size_t len,
    size_t* bytes_read);