#include "GPS.h"

void GPS::Init()
{
    Serial1.begin(9600);
    //Disable GPGSV messages by using the ublox protocol.
    uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
    Serial1.write(Disable_GPGSV, 11);
    delay(350);   //A small delay is added to give the GPS some time to respond @ 9600bps.
    //Set the refresh rate to 5Hz by using the ublox protocol.
    uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
    Serial1.write(Set_to_5Hz, 14);
    delay(350);   //A small delay is added to give the GPS some time to respond @ 9600bps.
    //Set the baud rate to 57.6kbps by using the ublox protocol.
    // uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
    //                            0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1
    //                           };
    // Serial1.write(Set_to_57kbps, 28);
    // delay(350);
    // Serial1.begin(57600);
    // delay(350);
}

void GPS::Read()
{
  //Place each GPS message in a line.
  while(Serial1.available()){
      char s1_byte = Serial1.read();
      // Serial.print(s1_byte);
      switch (s1_byte) {

          case '$':
          serial_byte_counter = 0;
          memset(NMEA_line, 0, sizeof (NMEA_line));
          break;

          case '*':
          if(NMEA_line[0] == 'G' && strlen(NMEA_line) > 10) new_line_found = 1;
          // Serial.println(Split(NMEA_line,',',0));
          break;

          default:
          NMEA_line[serial_byte_counter] = s1_byte;
          if(serial_byte_counter < 100) serial_byte_counter++;
          break;
      }
  }
  /*
$GNRMC,190339.20,A,3212.69713,N,03459.21116,E,0.049,,041118,,,A*6B
$GNVTG,,T,,M,0.049,N,0.090,K,A*39
$GNGGA,190339.20,3212.69713,N,03459.21116,E,1,08,1.08,97.0,M,17.7,M,,*79
$GNRMC,190339.40,A,3212.69713,N,03459.21112,E,0.254,,041118,,,A*67
$GNVTG,,T,,M,0.254,N,0.470,K,A*3D
$GNRMC,190339.60,A,3212.69713,N,03459.21110,E,0.284,,041118,,,A*6A
*01234567891123456789212345678931234567894123456789512345678961234567
$GNVTG,,T,,M,0.284,N,0.526,K,A*32

*01234567891123456789212345678931234567894123456789512345678961234567
$GNGGA,190339.60,3212.69713,N,03459.21110,E,1,08,1.08,97.5,M,17.7,M,,*7E
*/

//*01234567891123456789212345678931234567894123456789512345678961234567
//$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39
  if(new_line_found)
  {
    new_line_found = 0;
    if(NMEA_line[2] == 'G' && NMEA_line[3] == 'G' && NMEA_line[4] == 'A')
    {
      //Latitude
      float shift_dvider = 1.0f;
      float min_to_dec = NMEA_CharToInt(NMEA_line[18]) * 10;
      raw_latitude = NMEA_CharToInt(NMEA_line[16]) * 10;
      raw_latitude += NMEA_CharToInt(NMEA_line[17]);

      for(uint8_t i=19; i < 26; i++){
          if(i!= 20){
            min_to_dec += NMEA_CharToInt(NMEA_line[i]) / shift_dvider;
            shift_dvider *= 10;
          }
      }
      min_to_dec /= 60.0f;
      raw_latitude += min_to_dec;

      //Longitude
      raw_longitude = NMEA_CharToInt(NMEA_line[30]) * 10;
      raw_longitude += NMEA_CharToInt(NMEA_line[31]);
      min_to_dec = NMEA_CharToInt(NMEA_line[32]) * 10;
      shift_dvider = 1.0f;

      for(uint8_t i=32; i < 39; i++){
        if(i!= 34){
          min_to_dec += NMEA_CharToInt(NMEA_line[i]) / shift_dvider;
          shift_dvider *= 10;
          }
      }

      min_to_dec /= 60.0f;
      raw_longitude += min_to_dec;
      
      
      //Satellites
      _sats = NMEA_CharToInt(NMEA_line[45]) * 10;
      _sats += NMEA_CharToInt(NMEA_line[46]);
    }

    else if(NMEA_line[2] == 'G' && NMEA_line[3] == 'S' && NMEA_line[4] == 'A')
    {
      fix_t = NMEA_CharToInt(NMEA_line[8]);
    }


    //GPS hold
    if(gps_hold_flag)
    {
      gps_hold_flag = 0;
      lat_waypoint = raw_latitude;
      lon_waypoint = raw_longitude;
      waypoint_set = 1;
    }
    if(lat_waypoint)
    {
      gps_lat_error = raw_latitude*1000000 - lat_waypoint*1000000;
      gps_lon_error = raw_longitude*1000000 - lon_waypoint*1000000;


	    gps_pitch_adjust_north = (float)gps_lat_error * gps_p_gain + (float)(gps_lat_error - gps_lat_error_prev) * gps_d_gain;
	    gps_roll_adjust_north = (float)gps_lon_error * gps_p_gain + (float)(gps_lon_error - gps_lon_error_prev) * gps_d_gain;
	    
      gps_lat_error_prev = gps_lat_error;
	    gps_lon_error_prev = gps_lon_error;
      
    }
  }
}

float GPS::lon()
{
  return raw_longitude;
}

float GPS::lat()
{
  return raw_latitude;
}
uint8_t GPS::Fix()
{
  return fix_t;
}
uint8_t GPS::Active_satellites()
{
  return _sats;
}

int GPS::NMEA_CharToInt(char in)
{
  return (int)in - 48;
}