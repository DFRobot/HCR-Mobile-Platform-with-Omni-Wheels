#include "PID_v1.h"
//Encoder variables
const int Interval=10;
const byte encoder1pinA = 18;//A pin -> the interrupt pin 18
const byte encoder1pinB = 21;//B pin -> the digital pin 21
const byte encoder2pinA = 19;//A pin -> the interrupt pin 19
const byte encoder2pinB = 22;//B pin -> the digital pin 22
const byte encoder3pinA = 20;//A pin -> the interrupt pin 20
const byte encoder3pinB = 23;//B pin -> the digital pin 23
byte encoder1PinALast;
byte encoder2PinALast;
byte encoder3PinALast;
int duration1;//the number of the pulses of Moter1
int duration2;//the number of the pulses of Moter2
int duration3;//the number of the pulses of Moter3
int Speed1;
int Speed2;
int Speed3;
boolean Direction1;//the rotation Direction1 
boolean Direction2;//the rotation Direction1 
boolean Direction3;//the rotation Direction1 

//Motor Driver variables
int M1 = 2;     //M1 Direction Control
int M2 = 3;     //M2 Direction Control
int M3 = 4;     //M3 Direction Control
int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int E3 = 7;     //M3 Speed Control

//PID variables
const double Motor_2[3]={0.05,4,0.01};
double Setpoint1,Input1,Output1;
double Setpoint2,Input2,Output2;
double Setpoint3,Input3,Output3;
PID myPID1(&Input1,&Output1,&Setpoint1,Motor_2[0],Motor_2[1],Motor_2[2],DIRECT);
PID myPID2(&Input2,&Output2,&Setpoint2,Motor_2[0],Motor_2[1],Motor_2[2],DIRECT);
PID myPID3(&Input3,&Output3,&Setpoint3,Motor_2[0],Motor_2[1],Motor_2[2],DIRECT);
char val='s';

void setup()
{  
  Serial.begin(57600);//Initialize the serial port
  EncoderInit();//Initialize encoder
  int i; //Define output pin
  for(i=2;i<=7;i++) pinMode(i, OUTPUT);  
  digitalWrite(E1,LOW);   
  digitalWrite(E2,LOW);  
  digitalWrite(E3,LOW);  
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  myPID3.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-255,255);
  myPID2.SetOutputLimits(-255,255);
  myPID3.SetOutputLimits(-255,255);
 
}


void loop()
{
  
  if(Serial.available()) val = Serial.read();
   
    if(val != -1)
    {
      switch(val)
      {
      case 'w':                        
     PIDMovement (0,-200,200);       //Move forward
        break;
      case 'x':
      PIDMovement (0,200,-200);      //Move backward
        break;
      case 'a':                       //Turn Left
      PIDMovement (100,100,100);        
        break;       
      case 'd':                         //Turn Right
      PIDMovement (-100,-100,-100);
        break;
      case 'z':
        stop();
        break;
      case 's':
        PIDMovement (0,0,0);
        break;
      }
    }
    else stop();  
 
  Speed1=duration1*43/Interval;
  Speed2=duration2*43/Interval;
  Speed3=duration3*43/Interval;
  Serial.print("Val Value:");
  Serial.print(val);
  Serial.print("    Motor1Speed:");
  Serial.print(Speed1);
  Serial.print("    Motor2Speed:");
  Serial.print(Speed2);
  Serial.print("    Motor3Speed:");
  Serial.println(Speed3);
  duration1 = 0;
  duration2 = 0;
  duration3 = 0;
  delay(Interval);
}


//Motor modules
void stop(void)                 //Stop
{                 
  digitalWrite(E1,0); 
  digitalWrite(M1,LOW);    
  digitalWrite(E2,0);   
  digitalWrite(M2,LOW);    
  digitalWrite(E3,0);   
  digitalWrite(M3,LOW);   
  
}   

void Movement(int a,int b,int c)          
{
  if (a>=0) 
  {
    analogWrite (E1,a);      //PWM Speed Control
    digitalWrite(M1,HIGH);  
  }  
  else
  {
    analogWrite (E1,-a);      //PWM Speed Control
    digitalWrite(M1,LOW);  
  }
  if (b>=0) 
  {
    analogWrite (E2,b);       //PWM Speed Control
    digitalWrite(M2,HIGH);  
  }  
  else
  {
    analogWrite (E2,-b);     //PWM Speed Control
    digitalWrite(M2,LOW);  
  }
  if (c>=0) 
  {
    analogWrite (E3,c);      //PWM Speed Control
    digitalWrite(M3,HIGH);  
  }  
  else
  {
    analogWrite (E3,-c);     //PWM Speed Control
    digitalWrite(M3,LOW);  
  }
}


//PID modules
void PIDMovement(int a,int b,int c)
{
  Setpoint1=a;
  Setpoint2=b;
  Setpoint3=c;
  Input1=Speed1;
  Input2=Speed2;
  Input3=Speed3;
  myPID1.Compute();
  myPID2.Compute();
  myPID3.Compute();
  Movement (Output1,Output2,Output3);
}
   

//Encoder modules 

void EncoderInit() //Initialize encoder interruption
{
  Direction1 = true;//default -> Forward  
  Direction2 = true;//default -> Forward  
  Direction3 = true;//default -> Forward  
  pinMode(encoder1pinB,INPUT);  
  pinMode(encoder2pinB,INPUT);  
  pinMode(encoder3pinB,INPUT);  
  attachInterrupt(5, wheelSpeed1, CHANGE);
  attachInterrupt(4, wheelSpeed2, CHANGE);
  attachInterrupt(3, wheelSpeed3, CHANGE);
}
 
void wheelSpeed1()  //motor1 speed count
{
  int Lstate = digitalRead(encoder1pinA);
  if((encoder1PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder1pinB);
    if(val == LOW && Direction1)
    {
      Direction1 = false; //Reverse
    }
    else if(val == HIGH && !Direction1)
    {
      Direction1 = true;  //Forward
    }
  }
  encoder1PinALast = Lstate;
 
  if(!Direction1)  duration1++;
  else  duration1--;
}

void wheelSpeed2()  //motor2 speed count
{
  int Lstate = digitalRead(encoder2pinA);
  if((encoder2PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder2pinB);
    if(val == LOW && Direction2)
    {
      Direction2 = false; //Reverse
    }
    else if(val == HIGH && !Direction2)
    {
      Direction2 = true;  //Forward
    }
  }
  encoder2PinALast = Lstate;
 
  if(!Direction2)  duration2++;
  else  duration2--;
}

void wheelSpeed3()  //motor3 speed count
{
  int Lstate = digitalRead(encoder3pinA);
  if((encoder3PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder3pinB);
    if(val == LOW && Direction3)
    {
      Direction3 = false; //Reverse
    }
    else if(val == HIGH && !Direction3)
    {
      Direction3 = true;  //Forward
    }
  }
  encoder3PinALast = Lstate;
 
  if(!Direction3)  duration3++;
  else  duration3--;
}

