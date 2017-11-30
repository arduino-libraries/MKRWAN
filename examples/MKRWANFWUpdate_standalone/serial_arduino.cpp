#include "serial_arduino.h"


static port_err_t arduino_serial_open(struct port_interface *port) {
  port->dev->begin(port->ops->baudRate, port->ops->serial_mode);
  return PORT_ERR_OK;
}

static port_err_t arduino_close(struct port_interface *port) {
  return PORT_ERR_OK;
}

static port_err_t arduino_flush(struct port_interface *port) {
  port->dev->flush();
  return PORT_ERR_OK;
}

static port_err_t arduino_read(struct port_interface *port, void *buf, size_t nbyte) {
  uint8_t *pos = (uint8_t*)buf;
  while (nbyte) {
    if (port->dev->available()) {
      int c = port->dev->read();
      if (c < 0) {
        return PORT_ERR_UNKNOWN;
      }
      pos[0] = (uint8_t)c;
      nbyte--;
      pos++;
    }
  }
  return PORT_ERR_OK;
}

static port_err_t arduino_write(struct port_interface *port, void *buf, size_t nbyte) {
  port->dev->write((uint8_t*)buf, nbyte);
  return PORT_ERR_OK;
}

void assignCallbacks(struct port_interface *port) {
  port->open = arduino_serial_open;
  port->close = arduino_close;
  port->flush = arduino_flush;
  port->read = arduino_read;
  port->write = arduino_write;
}
