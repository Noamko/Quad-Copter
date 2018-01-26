#include <SoftwareSerial.h>

SoftwareSerial hc_12(7,6);

String serial_input;
String serial_output;
void setup() {
     pinMode(13,OUTPUT);
     Serial.begin(9600);
     hc_12.begin(9600);
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
       hc_12.print(serial_input);
  }
}

void GetTelemetry()
{
  if(hc_12.available())
  {
      int how_many = hc_12.available();
      char buffer[50];
      int c = hc_12.readBytes(buffer,how_many);
      Serial.print(buffer);
  }
}
