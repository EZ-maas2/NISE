#include <DynamixelSDK.h>
#include <math.h>



void setup()
{
  Serial.begin(9600);
}

void loop(){

  int sens0 = analogRead(A0);
  int sens1 = analogRead(A1);
  int sens2 = analogRead(A2);
  int sens3 = analogRead(A3);
  int sens4 = analogRead(A4);
  int sens5 = analogRead(A5);

  Serial.print(sens0);
  Serial.print(" ,");
  Serial.print(sens1);
  Serial.print(" ,");
  Serial.print(sens2);
  Serial.print(" ,");
  Serial.print(sens3);
  Serial.print(" ,");
  Serial.print(sens4);
  Serial.print(" ,");
  Serial.print(sens5);
  Serial.print(" ,");
  Serial.println();

  delay(20);
}