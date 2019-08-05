#include <SoftwareSerial.h>

SoftwareSerial bluetooth(10,11);

#define rele 3
#define pwm 5

String comando = "";
char junk;

void setup() {
//Serial.begin(38400);
bluetooth.begin(38400);

pinMode(rele, OUTPUT);
pinMode(pwm, OUTPUT);

}

void loop() {
comando = "";

if(bluetooth.available()){
  while(bluetooth.available()){
    char caracter = bluetooth.read();

    comando += caracter;
    delay(10);
  }

    while (bluetooth.available() > 0)  
    { junk = bluetooth.read() ; }

  if(comando == "a1") {

    if(digitalRead(rele) == HIGH){
      digitalWrite(rele, LOW);
      } else if(digitalRead(rele) == LOW){
      digitalWrite(rele, HIGH);
      }

  } else if (comando.substring(0,3) == "pwm") {

      analogWrite(pwm, 255/10*comando.substring(3).toInt());
      
  } 
}
}
