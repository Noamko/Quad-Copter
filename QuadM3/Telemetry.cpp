#include "Telemetry.h"

Telemetry::Telemetry()
{
  //Init variables
}

void Telemetry::Init(uint16_t baud)
{
  pinMode(PC14,OUTPUT);
  digitalWrite(AT_PIN,1);
  Serial1.begin(baud);
  baudrate = baud;
}

bool Telemetry::Receive()
{
  //receiving methods:
  /*
  1. fixed packet size - buffer will fill into a fixed size so we know when packed is fully received.
  2. packet will contain size info - first value of packet will be packet size & if size info is the same avaiable read it <---
  3. when data is available wait 3 seconds  then read it should be full
  4. read start char and end char to determine packet e2e
  5. if data available size is the same as prev loop probaply packet is full
  */

  packet_size = Serial1.available();
  if(packet_size > 0)
  {
    if(packet_size == prev_packet_size) checksum++;
    else checksum = 0;

    if(checksum >= 10) //TODO: change 10 after testing with long distance to a pre defined number
    {
      //Packet ready
      memset(in_buffer, 0, sizeof(in_buffer)); //Clear the buffer
      Serial1.readBytes(in_buffer,packet_size);
      // Transmit(in_buffer);
      return 1;
    }
    prev_packet_size = packet_size;
    return 0;
  } else return 0;
}

void Telemetry::Transmit(String data)
{
  char out_data[sizeof(data)];
  for(uint8_t i = 0; i < sizeof(data);i++)
  {
    out_data[i] = data[i];
  }
  Serial1.write(out_data);
}
void Telemetry::flush()
{
  Serial1.flush();
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