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

  Serial.print(sens0);
  Serial.print(" ,");
  Serial.print(sens1);
  Serial.print(" ,");
  Serial.print(sens2);
  Serial.print(" ,");
  Serial.println();

  delay(20);
}