#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "config.h"
#include <Arduino.h>

class Telemetry {
private:
  String getValue(String data, char separator, int index);
  String serial_input;

public:
  Telemetry ();
  void Init(uint32 baudrate);
  void Transmit(String buffer);
  float tlm_controller_sensativity;
  float autolevel_stregth;
  float _p;
  float _d;
  float _i;
  int8_t GetData(uint8_t data_available);
  uint8_t transmission_speed = 50;
  uint8_t tlm_transmit_flight_data;
  int16_t tlm_throttle_bias;
  uint8_t pid_setting;
};
#endif
