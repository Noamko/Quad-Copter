#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "config.h"
#include <Arduino.h>

class Telemetry {
private:
  uint16_t baudrate;
  uint16_t packet_size, prev_packet_size;
  uint8_t checksum = 0;
  


public:
  Telemetry ();
  void Init(uint16_t baud);
  bool Receive();
  void Transmit(String data);
  char in_buffer[100];
  void flush();
};
#endif
