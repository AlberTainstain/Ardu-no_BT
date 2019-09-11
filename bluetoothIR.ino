#include <SoftwareSerial.h>
#include <IRremote.h>

// DIRETIVAS DE COMPILAÇÃO
#define tempoTecla 350
#define frequencia 38 // kHz

SoftwareSerial bluetooth(10,11);

// DEFINIÇÃO DOS PINOS
//pino 3 biblioteca IR
#define rele1 4
#define rele2 6
#define pwm 5

String comando = "";
char junk;

// INFRAVERMELHO //
IRsend emissorIR;
unsigned int teclaBase[] = {9000,4450, 550,1600, 550,1700, 550,550, 550,550, 550,550, 550,550, 550,550, 550,1700, 550,550, 550,1600, 550,1600, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550, 550, 550,550, 550,550,550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550,550, 550};

int ilow = 550;
int ihigh = 1700;
///////////////////

void setup() {
//Serial.begin(38400);
bluetooth.begin(38400);

pinMode(rele1, OUTPUT);
pinMode(rele2, OUTPUT);
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

    if(digitalRead(rele1) == HIGH){
      digitalWrite(rele1, LOW);
      } else if(digitalRead(rele1) == LOW){
      digitalWrite(rele1, HIGH);
      }

  } else if (comando == "a2") {
    
    if(digitalRead(rele2) == HIGH){
      digitalWrite(rele2, LOW);
      } else if(digitalRead(rele2) == LOW){
      digitalWrite(rele2, HIGH);
      }
      
  } else if (comando.substring(0,3) == "pwm") {

      analogWrite(pwm, 25.5*comando.substring(3).toInt());
      
  }  else if(comando.substring(0,2) == "IR"){ //IR-1-1-16-4-16-1 (IR11F4F1)
        getDataArray(teclaBase, comando.substring(2,3).toInt(), comando.substring(3,4).toInt(), comando.substring(4,6).toInt(), comando.substring(6,7).toInt(), comando.substring(7,9).toInt(), comando.substring(9,10).toInt());
        emissorIR.sendRaw(teclaBase, 99, frequencia);
        bluetooth.println("Enviado:" + comando);
        bluetooth.println("Ligar:" + comando.substring(2,3));
        bluetooth.println("Func:" + comando.substring(3,4));
        bluetooth.println("Temp:" + comando.substring(4,6));
        bluetooth.println("Vel:" + comando.substring(6,7));
        bluetooth.println("Timer:" + comando.substring(7,9));
        bluetooth.println("Osc:" + comando.substring(9,10));
        delay(tempoTecla);
  }
}
}


//Ligar / Desligar : IR10000000
//Reset: IR00080000
//Ventilação : IR01000000
//Temp : IR00010000 19 / IR00090000 27
//Vel : IR00001000
//Osc : IR00000001
void getDataArray(unsigned int* array,  int ligar, int func, int temp, int vel, int timer, int osc){

  if(ligar == 1){
        array[39] = ihigh;
  } else {
        array[39] = ilow;
  }

  array[53] = ihigh; //RESFRIAR
  array[55] = ilow; //Ventilação

  array[59] = ilow;
  array[61] = ilow;
  array[63] = ilow;
  array[65] = ihigh; //26°C

  array[35] = ilow;
  array[37] = ilow; //Velocidade padrão

  array[57] = ilow; // timer off

///////////////////////////////////////////////////////FUNÇAO 1
  if(func == 0){ 
    //RESFRIAR
        array[53] = ihigh;

        // Temperatura  //////////////////////////
        if(bitRead(temp,0) == 0){
              array[59] = ilow; 
            } else {
              array[59] = ihigh;
            } 

            if(bitRead(temp,1) == 0){
              array[61] = ilow; 
            } else {
              array[61] = ihigh;
            } 

            if(bitRead(temp,2) == 0){
              array[63] = ilow; 
            } else {
              array[63] = ihigh;
            } 

            if(bitRead(temp,3) == 0){
              array[65] = ilow; 
            } else {
              array[65] = ihigh;
            } 
        // Velocidade 1, 2, 3 e 4 ////////////////
        
              if(bitRead(vel,0) == 0){
              array[35] = ilow; 
            } else {
              array[35] = ihigh;
            } 

              if(bitRead(vel,1) == 0){
              array[37] = ilow; 
            } else {
              array[37] = ihigh;
            } 
            
        // TIMER  ///////////////////////////////
        if(timer>0){
       array[57] = ihigh;

            if(bitRead(timer-1,0) == 0){
              array[67] = ilow; 
            } else {
              array[67] = ihigh;
            } 

            if(bitRead(timer-1,1) == 0){
              array[69] = ilow; 
            } else {
              array[69] = ihigh;
            } 

            if(bitRead(timer-1,2) == 0){
              array[71] = ilow; 
            } else {
              array[71] = ihigh;
            } 

            if(bitRead(timer-1,3) == 0){
              array[73] = ilow; 
            } else {
              array[73] = ihigh;
            } 
        }
        // Oscilação  ///////////////////////////
          if(func == 1){
               array[49] = ihigh;
          } else {
               array[49] = ilow;
          }

///////////////////////////////////////////////////////FUNÇAO 2
  } else if(func == 1){ 
    //VENTILAÇÃO
        array[53] = ilow;
        array[55] = ihigh;
        array[65] = ilow;

    //VELOCIDADE 1, 2 e 3
             if(bitRead(vel,0) == 0){
              array[35] = ilow; 
            } else {
              array[35] = ihigh;
            } 

              if(bitRead(vel,1) == 0){
              array[37] = ilow; 
            } else {
              array[37] = ihigh;
            } 

            if(vel == 0){ //Impedir velocidade padrão (00) ///
              array[35] = ihigh;
              array[37] = ihigh;
            }
        // TIMER  ///////////////////////////////
        if(timer>0){
       array[57] = ihigh;

            if(bitRead(timer-1,0) == 0){
              array[67] = ilow; 
            } else {
              array[67] = ihigh;
            } 

            if(bitRead(timer-1,1) == 0){
              array[69] = ilow; 
            } else {
              array[69] = ihigh;
            } 

            if(bitRead(timer-1,2) == 0){
              array[71] = ilow; 
            } else {
              array[71] = ihigh;
            } 

            if(bitRead(timer-1,3) == 0){
              array[73] = ilow; 
            } else {
              array[73] = ihigh;
            } 
        }
        // Oscilação  ///////////////////////////
          if(func == 1){
               array[49] = ihigh;
          } else {
               array[49] = ilow;
          }
  }
///////////////////////////////////////////////////////FUNÇAO END
}
