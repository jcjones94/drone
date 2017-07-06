


#include <RH_ASK.h>
#include <SPI.h>


RH_ASK driver;
int power = 0;
uint8_t data[2];

void setup() {

  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  if(!driver.init())
      Serial.println("init failed");
  
}

void loop() {
  digitalWrite(13, HIGH);
  data[0] = analogRead(A0)>>2;
  data[1] = analogRead(A1)>>2;
  data[2] = analogRead(A2)>>2;
  driver.send(data, 3);
  driver.waitPacketSent();
  delay(3);
  digitalWrite(13,LOW);
  delay(5);
}
