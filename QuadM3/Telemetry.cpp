#include "Telemetry.h"

Telemetry::Telemetry()
{
  //Init variables
}

void Telemetry::Init(uint32 baudrate)
{
  Serial1.begin(baudrate);
}

int8_t Telemetry::GetData(uint8_t data_available)
{
  //this is getting buggy at long buffers
  // int8_t how_many = Serial1.available();
  // char buffer[100];
  // int8_t c = Serial1.readBytes(buffer,50);
  // // Serial.println(String("da:") + data_available + "hm:"+how_many);
  // serial_input = buffer;

  serial_input = "";
  while(Serial1.available()){
    char c = Serial1.read();
    serial_input +=c;
    if(serial_input.length() > MAX_SERIAL_BUFFER)
    {
      break;
      return -1;
    }
    delay(5); //TODO: Check how this delay affect long distast transmission
  }
  String tlm_command = serial_input.substring(0,4);

  if(tlm_command == "$als")
  {
    float value = getValue(serial_input,',',1).toFloat();
    autolevel_stregth = value;
    return 2;
  }

  else if(tlm_command == "$csn")
  {
    // set controller sensativity
    float value = getValue(serial_input,',',1).toFloat();
    tlm_controller_sensativity = value;
    return 3;
  }
  else if(tlm_command == "$tfd")
  {
    // transmit flight data
    int8_t value = getValue(serial_input,',',1).toInt();
    tlm_transmit_flight_data = value;
    return 4;
  }

  else if(tlm_command == "$cgy")
  {
    // calibrate gyro
    return 5;
  }

  else if(tlm_command == "$gpg")
  {
    // send pid gains.
    return 6;
  }

  else if(tlm_command == "$pid")
  {
    //set new pid values
    pid_setting = getValue(serial_input,',',1).toInt();
    _p = getValue(serial_input,',',2).toFloat();
    _i = getValue(serial_input,',',3).toFloat();
    _d = getValue(serial_input,',',4).toFloat();
    return 7;
  }

  else if(tlm_command == "$rpv")
  {
    return 8;
  }

  else if(tlm_command == "$trs")
  {
    transmission_speed = getValue(serial_input,',',1).toInt();
    return -1;
  }

  else if (tlm_command == "test")
  {
    Transmit("test");
  }
}

String Telemetry::getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void Telemetry::Transmit(String buffer)
{
  Serial1.print(buffer + '\n');
}
