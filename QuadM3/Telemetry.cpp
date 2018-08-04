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
  packet_size = Serial1.available();
  if(packet_size > 0)
  {
    if(packet_size == prev_packet_size) checksum++;
    else checksum = 0;

    if(checksum >= 20) //TODO: change 10 after testing with long distance to a pre defined number
    {
      //Packet ready
      memset(in_buffer, 0, sizeof(in_buffer)); //Clear the buffer
      Serial1.readBytes(in_buffer,packet_size);
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