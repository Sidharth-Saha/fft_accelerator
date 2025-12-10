/* uart.c - Cross-platform UART implementation */
#include "uart.h"
#include <stdio.h>
#include <string.h>

static char error_buf[256] = { 0 };

static void set_error(const char* msg) 
{
    strncpy(error_buf, msg, sizeof(error_buf) - 1);
    error_buf[sizeof(error_buf) - 1] = '\0';
}

const char* uart_get_error(void) 
{
    return error_buf;
}

/* ============== Windows Implementation ============== */
#ifdef _WIN32

uart_handle_t uart_open(const char* port, const uart_config_t* config) 
{
    uart_config_t cfg = UART_CONFIG_DEFAULT;
    if (config) cfg = *config;

    /* Windows needs \\.\ prefix for COM ports >= 10 */
    char port_name[32];
    if (strncmp(port, "\\\\.\\", 4) == 0 || strncmp(port, "COM", 3) != 0) {
        snprintf(port_name, sizeof(port_name), "%s", port);
    }
    else {
        snprintf(port_name, sizeof(port_name), "\\\\.\\%s", port);
    }

    HANDLE handle = CreateFileA(
        port_name,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );

    if (handle == INVALID_HANDLE_VALUE) {
        snprintf(error_buf, sizeof(error_buf),
            "Failed to open %s (error %lu)", port, GetLastError());
        return UART_INVALID_HANDLE;
    }

    /* Configure port */
    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(DCB);

    if (!GetCommState(handle, &dcb)) {
        set_error("Failed to get COM state");
        CloseHandle(handle);
        return UART_INVALID_HANDLE;
    }

    dcb.BaudRate = cfg.baud_rate;
    dcb.ByteSize = cfg.data_bits;

    switch (cfg.parity) {
    case UART_PARITY_NONE: dcb.Parity = NOPARITY; break;
    case UART_PARITY_ODD:  dcb.Parity = ODDPARITY; break;
    case UART_PARITY_EVEN: dcb.Parity = EVENPARITY; break;
    }

    dcb.StopBits = (cfg.stop_bits == UART_STOP_2) ? TWOSTOPBITS : ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = (cfg.parity != UART_PARITY_NONE);
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;

    if (!SetCommState(handle, &dcb)) {
        set_error("Failed to set COM state");
        CloseHandle(handle);
        return UART_INVALID_HANDLE;
    }

    /* Set timeouts */
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = cfg.timeout_ms;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = cfg.timeout_ms;
    timeouts.WriteTotalTimeoutMultiplier = 0;

    if (!SetCommTimeouts(handle, &timeouts)) {
        set_error("Failed to set timeouts");
        CloseHandle(handle);
        return UART_INVALID_HANDLE;
    }

    /* Clear buffers */
    PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);

    return handle;
}

void uart_close(uart_handle_t handle) 
{
    if (handle != UART_INVALID_HANDLE) {
        CloseHandle(handle);
    }
}

int uart_write(uart_handle_t handle, const void* data, size_t len) 
{
    DWORD written = 0;
    if (!WriteFile(handle, data, (DWORD)len, &written, NULL)) {
        set_error("Write failed");
        return -1;
    }
    return (int)written;
}

int uart_read(uart_handle_t handle, void* buf, size_t len) 
{
    DWORD bytes_read = 0;
    if (!ReadFile(handle, buf, (DWORD)len, &bytes_read, NULL)) {
        set_error("Read failed");
        return -1;
    }
    return (int)bytes_read;
}

int uart_flush(uart_handle_t handle) 
{
    if (!PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR)) {
        set_error("Flush failed");
        return -1;
    }
    return 0;
}

/* ============== Linux/POSIX Implementation ============== */
#else

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

static speed_t get_baud_constant(uart_baud_t baud) 
{
    switch (baud) {
    case UART_BAUD_9600:   return B9600;
    case UART_BAUD_19200:  return B19200;
    case UART_BAUD_38400:  return B38400;
    case UART_BAUD_57600:  return B57600;
    case UART_BAUD_115200: return B115200;
    case UART_BAUD_230400: return B230400;
    case UART_BAUD_460800: return B460800;
    case UART_BAUD_921600: return B921600;
    default:               return B115200;
    }
}

uart_handle_t uart_open(const char* port, const uart_config_t* config) 
{
    uart_config_t cfg = UART_CONFIG_DEFAULT;
    if (config) cfg = *config;

    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        snprintf(error_buf, sizeof(error_buf),
            "Failed to open %s: %s", port, strerror(errno));
        return UART_INVALID_HANDLE;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        set_error("Failed to get terminal attributes");
        close(fd);
        return UART_INVALID_HANDLE;
    }

    /* Set baud rate */
    speed_t baud = get_baud_constant(cfg.baud_rate);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    /* Data bits */
    tty.c_cflag &= ~CSIZE;
    switch (cfg.data_bits) {
    case 5: tty.c_cflag |= CS5; break;
    case 6: tty.c_cflag |= CS6; break;
    case 7: tty.c_cflag |= CS7; break;
    default: tty.c_cflag |= CS8; break;
    }

    /* Parity */
    switch (cfg.parity) {
    case UART_PARITY_NONE:
        tty.c_cflag &= ~PARENB;
        break;
    case UART_PARITY_ODD:
        tty.c_cflag |= PARENB | PARODD;
        break;
    case UART_PARITY_EVEN:
        tty.c_cflag |= PARENB;
        tty.c_cflag &= ~PARODD;
        break;
    }

    /* Stop bits */
    if (cfg.stop_bits == UART_STOP_2) {
        tty.c_cflag |= CSTOPB;
    }
    else {
        tty.c_cflag &= ~CSTOPB;
    }

    /* Other flags */
    tty.c_cflag &= ~CRTSCTS;           /* No hardware flow control */
    tty.c_cflag |= CREAD | CLOCAL;     /* Enable receiver, ignore modem */

    /* Raw input */
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);

    /* Disable software flow control */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    /* Raw output */
    tty.c_oflag &= ~(OPOST | ONLCR);

    /* Timeout: VTIME is in tenths of a second */
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = (cfg.timeout_ms + 99) / 100;  /* Round up */
    if (tty.c_cc[VTIME] == 0) tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        set_error("Failed to set terminal attributes");
        close(fd);
        return UART_INVALID_HANDLE;
    }

    /* Flush buffers */
    tcflush(fd, TCIOFLUSH);

    return fd;
}

void uart_close(uart_handle_t handle) 
{
    if (handle != UART_INVALID_HANDLE) {
        close(handle);
    }
}

int uart_write(uart_handle_t handle, const void* data, size_t len) 
{
    ssize_t written = write(handle, data, len);
    if (written < 0) {
        snprintf(error_buf, sizeof(error_buf), "Write failed: %s", strerror(errno));
        return -1;
    }
    return (int)written;
}

int uart_read(uart_handle_t handle, void* buf, size_t len) 
{
    ssize_t bytes_read = read(handle, buf, len);
    if (bytes_read < 0) {
        snprintf(error_buf, sizeof(error_buf), "Read failed: %s", strerror(errno));
        return -1;
    }
    return (int)bytes_read;
}

int uart_flush(uart_handle_t handle) 
{
    if (tcflush(handle, TCIOFLUSH) != 0) {
        snprintf(error_buf, sizeof(error_buf), "Flush failed: %s", strerror(errno));
        return -1;
    }
    return 0;
}

#endif /* _WIN32 */