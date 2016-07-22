#include "PID_v1.h"
#include <Metro.h> 
#include "GoBLE.h"
#include <Math.h>

//Encoder variables
const int Interval=10;                                     //Encoder data refresh time interval
const byte encoder1pinA = 18;                              //A pin -> the interrupt pin 18
const byte encoder1pinB = 21;                              //B pin -> the digital pin 21
const byte encoder2pinA = 19;                              //A pin -> the interrupt pin 19
const byte encoder2pinB = 22;                              //B pin -> the digital pin 22
const byte encoder3pinA = 20;                              //A pin -> the interrupt pin 20
const byte encoder3pinB = 23;                              //B pin -> the digital pin 23
byte encoder1PinALast;
byte encoder2PinALast;
byte encoder3PinALast;
int duration1;                                  //the number of the pulses of Moter1 in the interval
int duration2;                                  //the number of the pulses of Moter2 in the interval
int duration3;                                  //the number of the pulses of Moter3 in the interval
int Speed1;                                     //actual speed of motor1
int Speed2;                                     //actual speed of motor2 
int Speed3;                                     //actual speed of motor3
double SpeedX;                                     //actual speed of along X axis
double SpeedY;                                     //actual speed of along Y axis
double SpeedZ;                                     //actual speed of along Z axis
int SpeedInput1;
int SpeedInput2;
int SpeedInput3;
boolean Direction1;                              //the rotation Direction1 
boolean Direction2;                              //the rotation Direction2 
boolean Direction3;                              //the rotation Direction3

//Motor Driver variables
int M1 = 2;     //M1 Direction Control
int M2 = 3;     //M2 Direction Control
int M3 = 4;     //M3 Direction Control
int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int E3 = 7;     //M3 Speed Control

//PID variables
const double Motor_2[3]={0,2,0.03};                //PID parameters [P,I,D]
double Setpoint1,Input1,Output1;                   //PID input&output values for Motor1
double Setpoint2,Input2,Output2;                   //PID input&output values for Motor2
double Setpoint3,Input3,Output3;                   //PID input&output values for Motor3
PID myPID1(&Input1,&Output1,&Setpoint1,Motor_2[0],Motor_2[1],Motor_2[2],DIRECT);  
PID myPID2(&Input2,&Output2,&Setpoint2,Motor_2[0],Motor_2[1],Motor_2[2],DIRECT);
PID myPID3(&Input3,&Output3,&Setpoint3,Motor_2[0],Motor_2[1],Motor_2[2],DIRECT);
char val='x';

//GoBLE Variables

void setup()
{  
  Goble.begin();
//  Serial.begin(57600);//Initialize the serial port
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
  int joystickX, joystickY;
  int buttonState[6];
  int Turn=0;                                     //actual speed of along Y axis
  if(Goble.available()){
    joystickX = Goble.readJoystickX();
    joystickY = Goble.readJoystickY();
    buttonState[SWITCH_UP]     = Goble.readSwitchUp();
    buttonState[SWITCH_DOWN]   = Goble.readSwitchDown();
    buttonState[SWITCH_LEFT]   = Goble.readSwitchLeft();
    buttonState[SWITCH_RIGHT]  = Goble.readSwitchRight();
    buttonState[SWITCH_SELECT] = Goble.readSwitchSelect();
    buttonState[SWITCH_START]  = Goble.readSwitchStart();
    if (buttonState[2] == PRESSED)   Turn=1;
    if (buttonState[4] == PRESSED)   Turn=-1;
//    Serial.print(" X:");
//    Serial.print(joystickX);
//    Serial.print("  Y:");
//    Serial.print(joystickY);
//    for (int i = 1; i < 7; i++) 
//    {
//      Serial.print("  B");
//      Serial.print(i);
//      Serial.print(":");
//      if (buttonState[i] == PRESSED)   Serial.print("On ");
//      if (buttonState[i] == PRESSED)  Serial.print("Off");
//    }

    SpeedInput1=(double(joystickY-128)+Turn*100)*1;
    SpeedInput2=(0.866025 *double(joystickX-128)-0.5*double(joystickY-128)+Turn*100)*1;
    SpeedInput3=(-0.866025 *double(joystickX-128)-0.5*double(joystickY-128)+Turn*100)*1;
//    Serial.print("  Input1:");
//    Serial.print( SpeedInput1);
//    Serial.print("  Input2:");
//    Serial.print( SpeedInput2);
//    Serial.print("  Input3:");
//    Serial.print( SpeedInput3);
//    Serial.println("");
  }
  PIDMovement (SpeedInput1,SpeedInput2,SpeedInput3);       //sets HCR to be moving in required state
  Speed1=duration1*43/Interval;                            //calculates the actual speed of motor1, constant 43 for unifing the speed to the PWM input value 
  Speed2=duration2*43/Interval;                            //calculates the actual speed of motor1, constant 43 for unifing the speed to the PWM input value
  Speed3=duration3*43/Interval;                            //calculates the actual speed of motor1, constant 43 for unifing the speed to the PWM input value
  SpeedX=0.57735*Speed2-0.57735*Speed3;                    //calculates the actual speed alone X axis
  SpeedY=0.666667*Speed1-0.333333*Speed2-0.333333*Speed3;                 //calculates the actual speed along Y axis  
//  Serial.print("Val:");                                  //speed serial print
//  Serial.print(val);
//  Serial.print("  M1:");
//  Serial.print(Speed1);
//  Serial.print("  M2:");
//  Serial.print(Speed2);
//  Serial.print("  M3:");
//  Serial.print(Speed3);
//  Serial.print("  X:");
//  Serial.print(SpeedX);
//  Serial.print("  Y:");
//  Serial.print(SpeedY);
//  Serial.println("");
  duration1 = 0;                                            //reset duration1 for the next counting cycle
  duration2 = 0;                                            //reset duration2 for the next counting cycle
  duration3 = 0;                                            //reset duration3 for the next counting cycle
  delay(Interval);     
  //delay some certain time for aquiring pulse from encoder
}


//Motor modules
void stop(void) 
{                   //停止
  digitalWrite(E1,0); 
  digitalWrite(M1,LOW);    
  digitalWrite(E2,0);   
  digitalWrite(M2,LOW);    
  digitalWrite(E3,0);   
  digitalWrite(M3,LOW);   
  
}   

//move without PWM control
void Movement(int a,int b,int c)        
{
  if (a>=0) 
  {
    analogWrite (E1,a);      //motor1 move forward at speed a
    digitalWrite(M1,HIGH);  
  }  
  else
  {
    analogWrite (E1,-a);      //motor1 move backward at speed a
    digitalWrite(M1,LOW);  
  }
  if (b>=0) 
  {
    analogWrite (E2,b);      //motor2 move forward at speed b
    digitalWrite(M2,HIGH);  
  }  
  else
  {
    analogWrite (E2,-b);     //motor2 move backward at speed b
    digitalWrite(M2,LOW);  
  }
  if (c>=0) 
  {
    analogWrite (E3,c);      //motor3 move forward at speed c
    digitalWrite(M3,HIGH);  
  }  
  else
  {
    analogWrite (E3,-c);      //motor3 move backward at speed c
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

