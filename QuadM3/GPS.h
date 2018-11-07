#ifndef GPS_H
#define GPS_H

#include "config.h"

class GPS
{
private:
    char serial_byte, NMEA_line[100];
    uint8_t _sats, fix_t;
    uint8_t waypoint_set, latitude_north, longiude_east ;
    uint8_t serial_byte_counter;
    float raw_latitude, raw_longitude;
    float _hdop, _vdop;
    uint8_t new_line_found;
    char* getValue(String data, char separator, uint16_t index);
    int NMEA_CharToInt(char in);


public:
    void Init();
    void Read();
    float lon();
    float lat();
    uint8_t Fix();
    uint8_t Active_satellites();
};

#endif