#ifndef __SERIAL_PORT_H__
#define __SERIAL_PORT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <sys/types.h>

/// Opens the specified serial port, sets it up for binary communication,
/// configures its read timeouts, and sets its baud rate.
/// Returns a non-negative file descriptor on success, or -1 on failure.
int open_serial_port(const char *device, uint32_t baud_rate);

/// Writes bytes to the serial port, returning 0 on success and -1 on failure.
int write_port(int fd, uint8_t *buffer, size_t size);

/// Reads bytes from the serial port.
/// Returns after all the desired bytes have been read, or if there is a
/// timeout or other error.
/// Returns the number of bytes successfully read into the buffer, or -1 if
/// there was an error reading.
ssize_t read_port(int fd, uint8_t *buffer, size_t size);

/// closes already opened serial port
void close_serial_port(int fd);

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_PORT_H__ */