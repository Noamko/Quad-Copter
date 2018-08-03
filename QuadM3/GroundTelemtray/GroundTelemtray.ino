// #include <SoftwareSerial.h>

// SoftwareSerial hc_12(4,3);

String serial_input;
String serial_output;

uint16_t baudrate;
uint16_t packet_size, prev_packet_size;
uint8_t checksum = 0;
char in_buffer[100];

void setup() {
     pinMode(5,OUTPUT);
     Serial1.begin(1200);
     Serial.begin(9600);
     digitalWrite(5,1);
}

void loop() {
  GetCommands();
  GetTelemetry();
}

void GetCommands()
{
  if(Serial.available())
  {
       serial_input = "";
       while(Serial.available())
       {
            char c = Serial.read();
            serial_input += c;
            delay(10);
       }
       Serial1.flush();
       Serial1.print(serial_input);
  }
}

void GetTelemetry()
{
  packet_size = Serial1.available();
  if(packet_size > 0)
  {
    if(packet_size == prev_packet_size) checksum++;
    else checksum = 0;

    if(checksum >= 10)
    {
      memset(in_buffer,0,sizeof(in_buffer));
      Serial1.readBytes(in_buffer,packet_size);
      Serial.print((char*)in_buffer);
    }
    prev_packet_size = packet_size;
  }
}
/*
bool Telemetry::Receive()
{
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

      return 1;
    }
    prev_packet_size = packet_size;
    return 0;
  } else return 1;
}
*/