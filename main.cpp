
#include <Arduino.h>
#include <PID_v1.h>
#include <stdio.h>
#include <Servo.h>

//Definicion de los pines a utilizar en el proyecto
#define  POTENCIOMETRO1  A0
#define PWM1 9
#define VELOCIDAD_DERECHA 85.5  //85.5
#define VELOCIDAD_IZQ 98  //99
#define PARO 90

//Declaracion de funciones propias:
void irPosicion( Servo servo, int pos_f , int pos_ini , int lec , int pin);



//Declaracion de las variables globales
int lecturaPotenciometro1;      //variable que almacena la lectura analógica
int position1;   //posicion del potenciometro en tanto por ciento
Servo servo1;



void setup() {
  Serial.begin(9600);
  servo1.attach(PWM1);
  lecturaPotenciometro1=analogRead(POTENCIOMETRO1);      //variable que almacena la lectura analógica
  position1=map(lecturaPotenciometro1, 350,1023,0,180);
}

void loop() {




  /*  irPosicion(servo1, 0, position1, lecturaPotenciometro1,POTENCIOMETRO1);
    delay(3000);
    irPosicion(servo1, 90, position1, lecturaPotenciometro1,POTENCIOMETRO1);
    delay(3000);
    irPosicion(servo1, 180, position1, lecturaPotenciometro1,POTENCIOMETRO1);
    delay(3000);
*/
for(int i=0; i<=180; i){
  irPosicion(servo1, i, position1, lecturaPotenciometro1,POTENCIOMETRO1);
  delay(1000);
  i+=10;
}




}


//Funciones propias:
void irPosicion( Servo servo, int pos_f, int pos_ini, int lectura, int pin){

  if(pos_ini>pos_f){
    while(pos_ini>pos_f){
      lectura = analogRead(pin);          // realizar la lectura analógica raw

      //Dado que el robot solo permite girar 180 al potenciometro, he tomado el
      //valor mas pequeño de este que corresponde a 3,4 ohm y un int de 355.
      pos_ini=map(lectura, 350,1023,0,180);
      //Imprimir por monitor serie
      Serial.println(pos_ini);
      servo.write(VELOCIDAD_IZQ);
      //delay(10);
    }
    servo.write(PARO); return;
  }

  else if(pos_ini<pos_f){
    while(pos_ini<pos_f){
      lectura = analogRead(pin);          // realizar la lectura analógica raw
      pos_ini=map(lectura, 350,1023,0,180);
      //Imprimir por monitor serie
      Serial.println(pos_ini);
      servo.write(VELOCIDAD_DERECHA);
      //delay(10);
    } servo.write(PARO); return;

  }

  else if(pos_ini==pos_f){
    servo.write(PARO);
    return;
  }

}





/*
int ControlTheory();



  double Error, Time, PreviousTime, PreviousError, LastPosition, Proportional, Integral, Derivative, DutyCycle, ActualPosition;
  int Drive, ScaleFactor, IntThresh, Motor, P, I, D, dt;

  int Pot = A4;
  Servo servo1;

  const int analogPin = A0;
  int value;      //variable que almacena la lectura analógica raw
  int position;   //posicion del potenciometro en tanto por ciento

  double Kp = 1;
  double Ki = 1;
  double Kd = 0;

  double SetPoint, Input, Output;

  PID myPID(&Input, &Output, &SetPoint, Kp, Ki, Kd, DIRECT);

  void setup()
  {
    Serial.begin(9600);
    myPID.SetMode(MANUAL);
    servo1.attach(9);

  }
  void loop()
  {
     value = analogRead(analogPin);          // realizar la lectura analógica raw
     position = map(value, 0, 10023, 0, 100);
     // convertir a porcentaje
     //Imprimir por monitor serie
     Serial.println(position);

    SetPoint = 20;
    ActualPosition = position;
    myPID.Compute();

    Time = millis();

    ControlTheory();

    Error = SetPoint - ActualPosition;

    if ( abs(Error) <= 1){
      Error = 0;//Señal de potenciometro bastante inestable
                //para que no oscile cuando esta muy cerca del valor, poner directamente error 0 para que el servo se pare
    }
    P = Error*Kp;
    I = Integral*Ki;
    D = (LastPosition - ActualPosition)*Kd;
    Drive = P + I + D;

    if (Drive < 0)
    {
      servo1.write(86);//Sentido -
    }
    else if (Drive == 0)
    {
      servo1.write(90);//Parado
    }
    else if (Drive > 0)
    {
      servo1.write(98);//Sentido +
    }

    LastPosition = ActualPosition;

    //printPIDResults();
  }

  int ControlTheory()
  {
    Error = SetPoint - ActualPosition;
    Proportional = Error*Kp;
    Integral = Integral+(Error*dt);
    Derivative = (Error-PreviousError)/dt;
    Drive = (Proportional)+(Integral*Ki)+(Derivative*Kd);
    PreviousError = Error;

    return  Proportional + Integral + Derivative;
  }

  void printPIDResults()
  {
    Serial.print("PID formula (P + I + D): ");
    Serial.print(Proportional, 2);
    Serial.print(" + ");
    Serial.print(Integral, 2);
    Serial.print(" + ");
    Serial.println(Derivative, 2);
  }

*/


    /*servoRotCont.write(86);//clockwise
    delay(500);
    servoRotCont.write(90); //stop (el valor 90 depende del motor.
    //Es conveniente probar valores por encima o por debajo
    //de 90 hasta comprobar que se para el servomotor.
    delay(500);
    servoRotCont.write(97);//counter-clockwise
    delay(1000);
    servoRotCont.write(90); //stop
    delay(500);
    */
