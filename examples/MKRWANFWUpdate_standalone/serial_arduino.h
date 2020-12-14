#ifndef _H_PORT
#define _H_PORT

#include "Arduino.h"

#define usleep(x)  delayMicroseconds(x)

#ifndef __MBED__
#include "Uart.h"
#define UART Uart
#endif

#define fprintf(output, ...) {                                                \
                              do {                                            \
                                 char string[100];                            \
                                 sprintf (string, __VA_ARGS__);               \
                                 Serial.print(string);                        \
                              } while (0);                                    \
                             }

/* flags */
#define PORT_BYTE  (1 << 0)  /* byte (not frame) oriented */
#define PORT_GVR_ETX  (1 << 1)  /* cmd GVR returns protection status */
#define PORT_CMD_INIT (1 << 2)  /* use INIT cmd to autodetect speed */
#define PORT_RETRY  (1 << 3)  /* allowed read() retry after timeout */
#define PORT_STRETCH_W  (1 << 4)  /* warning for no-stretching commands */

/* all options and flags used to open and configure an interface */
struct port_options {
  int baudRate;
  int serial_mode;
};

/*
 * Specify the length of reply for command GET
 * This is helpful for frame-oriented protocols, e.g. i2c, to avoid time
 * consuming try-fail-timeout-retry operation.
 * On byte-oriented protocols, i.e. UART, this information would be skipped
 * after read the first byte, so not needed.
 */
struct varlen_cmd {
  uint8_t version;
  uint8_t length;
};

typedef enum {
  PORT_ERR_OK = 0,
  PORT_ERR_NODEV,   /* No such device */
  PORT_ERR_TIMEDOUT,  /* Operation timed out */
  PORT_ERR_UNKNOWN,
} port_err_t;

struct port_interface {
  const char *name;
  unsigned flags;
  port_err_t (*open)(struct port_interface *port);
  port_err_t (*close)(struct port_interface *port);
  port_err_t (*flush)(struct port_interface *port);
  port_err_t (*read)(struct port_interface *port, void *buf, size_t nbyte);
  port_err_t (*write)(struct port_interface *port, void *buf, size_t nbyte);
  struct varlen_cmd *cmd_get_reply = NULL;
  UART *dev;
  struct port_options *ops;
};

void assignCallbacks(struct port_interface *port);


#endif
