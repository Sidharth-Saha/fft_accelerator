/* uart.h - Cross-platform UART abstraction */
#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef _WIN32
#include <windows.h>
typedef HANDLE uart_handle_t;
#define UART_INVALID_HANDLE INVALID_HANDLE_VALUE
#else
typedef int uart_handle_t;
#define UART_INVALID_HANDLE (-1)
#endif

typedef enum {
    UART_BAUD_9600 = 9600,
    UART_BAUD_19200 = 19200,
    UART_BAUD_38400 = 38400,
    UART_BAUD_57600 = 57600,
    UART_BAUD_115200 = 115200,
    UART_BAUD_230400 = 230400,
    UART_BAUD_460800 = 460800,
    UART_BAUD_921600 = 921600
} uart_baud_t;

typedef enum {
    UART_PARITY_NONE,
    UART_PARITY_ODD,
    UART_PARITY_EVEN
} uart_parity_t;

typedef enum {
    UART_STOP_1,
    UART_STOP_2
} uart_stop_t;

typedef struct {
    uart_baud_t   baud_rate;
    uint8_t       data_bits;    /* 5, 6, 7, or 8 */
    uart_parity_t parity;
    uart_stop_t   stop_bits;
    uint32_t      timeout_ms;   /* Read timeout in milliseconds */
} uart_config_t;

/* Default configuration: 115200 8N1, 1 second timeout */
#define UART_CONFIG_DEFAULT { \
    .baud_rate = UART_BAUD_115200, \
    .data_bits = 8, \
    .parity = UART_PARITY_NONE, \
    .stop_bits = UART_STOP_1, \
    .timeout_ms = 1000 \
}

/**
 * Open a UART port
 * @param port Port name (e.g., "/dev/ttyUSB0" on Linux, "COM3" on Windows)
 * @param config Pointer to configuration, or NULL for defaults
 * @return Handle on success, UART_INVALID_HANDLE on failure
 */
uart_handle_t uart_open(const char* port, const uart_config_t* config);

/**
 * Close a UART port
 */
void uart_close(uart_handle_t handle);

/**
 * Write data to UART
 * @return Number of bytes written, or -1 on error
 */
int uart_write(uart_handle_t handle, const void* data, size_t len);

/**
 * Read data from UART
 * @return Number of bytes read, 0 on timeout, or -1 on error
 */
int uart_read(uart_handle_t handle, void* buf, size_t len);

/**
 * Flush receive and transmit buffers
 */
int uart_flush(uart_handle_t handle);

/**
 * Get last error message
 */
const char* uart_get_error(void);