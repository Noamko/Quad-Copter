#ifndef GPS_H
#define GPS_H

#include "config.h"

class GPS
{
private:
    uint8_t _sats, fix_t;
    uint8_t waypoint_set, latitude_north, longiude_east ;
    uint8_t serial_byte_counter;
    float raw_latitude, raw_longitude;
    float _hdop, _vdop;
    uint8_t new_line_found;
    int NMEA_CharToInt(char in);
    char serial_byte, NMEA_line[100];
    int32_t gps_lat_error,gps_lon_error;
    int32_t gps_lat_error_prev, gps_lon_error_prev;
    float gps_p_gain = 2.7;
    float gps_d_gain = 6.5;
    float lat_waypoint,lon_waypoint;

public:
    void Init();
    void Read();
    float lon();
    float lat();
    uint8_t Fix();
    uint8_t Active_satellites();
    float gps_pitch_adjust_north, gps_roll_adjust_north;
    bool gps_hold_flag = 0;
};

#endif