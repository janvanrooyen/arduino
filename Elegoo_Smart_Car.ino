#include <boarddefs.h>
#include <IRremote.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>


#include <NewPing.h>
#include <Servo.h> //servo library

Servo myservo; // create servo object to control servo //distance sensor

//int Echo = A4;  //distance sensor
//int Trig = A5; //distance sensor
int receiverpin = 12; //remote control IR sensor reciever
int in1=9;
int in2=8;
int in3=7;
int in4=6;
int ENA=10;
int ENB=5;
int ABS=200;
unsigned long RED; //remote control value
unsigned long lastButtonTime = millis(); //used for remote control
int timeoutDelay = 400; 
int rightDistance = 0,leftDistance = 0,middleDistance = 0; //distance sensor

#define TRIGGER_PIN  A5
#define ECHO_PIN     A4
#define MAX_DISTANCE 200

#define A 1464047974 //Forward Up arrow on remote control
#define AA 1464047719 //Hold Forward Up Arrow on remote control

#define B 1464061234 //Back Down arrow on remote control

#define X 1464030379 //Stop Middle button on remote control

#define XX 1464030634 //Stop Middle button on remote control (held down)

#define C 1464039814 //Left Left arrow on remote control

#define D 1464055114 //Right right arrow on remote control

#define PLAY 1464021709 //Play button on remote control

IRrecv irrecv(receiverpin); //remote control reciever
decode_results results; //remote control reciever

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void _mBack()
{ 
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  Serial.println("go back!");
}
void _mForward()
{
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  Serial.println("go forward!");
}  
void _mleft()
{
  analogWrite(ENA,ABS);
  analogWrite(ENB,ABS);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  Serial.println("go left!");
}
void _mright()
{
  analogWrite(ENA,ABS);
  analogWrite(ENB,ABS);
  digitalWrite(in1, HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  Serial.println("go right!");
}
void _mStop()
{
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);
//  Serial.println("STOP!");  
}
void _middleDistance()
{
  myservo.write(90);
  delay(500);
  middleDistance=Distance_test();
  Serial.println("Middle Distance = ");
  Serial.println(middleDistance);
}
void _rightDistance()
{
  myservo.write(10);
  delay(500);
  rightDistance=Distance_test();
  Serial.println("Right Distance = ");
  Serial.println(rightDistance);  
}
void _leftDistance()
{
  myservo.write(180);
  delay(500);
  leftDistance=Distance_test();
  Serial.println("Left Distance = ");
  Serial.println(leftDistance);
}

 /*Ultrasonic distance measurement debug function */
int Distance_test()   
{
  Serial.print("ping:");
  Serial.print(sonar.ping_cm());
  Serial.println("cm");
} 

void setup() {
  //setup code, runs once
  myservo.attach(3);// attach servo on pin 3 to servo object
  pinMode(ECHO_PIN, INPUT);    //distance sensor
  pinMode(TRIGGER_PIN, OUTPUT);  //distance sensor
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(receiverpin,INPUT); //remote control
  Serial.begin(9600);
  _mStop();
  irrecv.enableIRIn(); //enables remote control
}

void loop() {

  if (irrecv.decode(&results)) //if a button is pushed on the remote control
  { 
    lastButtonTime=millis();
    RED=results.value;
    irrecv.resume();
    delay(150);
    if (RED==PLAY)
    {
      myservo.write(90);
      _middleDistance();
      delay(500);
      _rightDistance();
      delay(500);
      _leftDistance();
      delay(500);
      myservo.write(90);
    }
    if (RED==A || RED==AA)
    {
      if (sonar.ping_cm()<20)
        {
        _mStop();
        }
        else
        {
        _mForward();  
        }     
    }
    if (RED==B)
    {
      _mBack();
    }
    else if(RED==C)
    {
      _mleft();
    }
    else if(RED==D)
    {
      _mright();
    }
    else if(RED==X)
    { 
      _mStop();
    }
    else if(RED==XX)
    { 
      _mStop();
    }
  }
  else if (millis()-lastButtonTime>timeoutDelay){
    _mStop();
    //Serial.println("No input, All Stop");
    }
} 

