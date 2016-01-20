/**
 * Copyright (C), 2015 - 2016, ChenZiping
 */
 
#include "MeOrion.h"
#include <Servo.h>
#include <SoftwareSerial.h>

unsigned char resetDegree[5] = {90,35,25,20};
unsigned char currentDegree[5] = {90,35,25,20};

int state = 0;

MeDCMotor pawArm(PORT_1);
MeDCMotor leftMotor(M1),rightMotor(M2);
MePort port1(PORT_3),//1:底座旋转;2:底座俯仰
       port2(PORT_4);//1：中间上下;2:上部上下
Servo bottomArm1,bottomArm2,middleArm1,middleArm2,topArm;
MeUltrasonicSensor ultraSensor(PORT_7); 
MeUltrasonicSensor ultraSensor1(PORT_8); 
MeInfraredReceiver infraredReceiverDecode(PORT_6);

int16_t servo1Pin =  port1.pin1();
int16_t servo2Pin =  port1.pin2();
int16_t servo3Pin =  port2.pin1();
int16_t servo4Pin =  port2.pin2();
//int16_t servo5Pin =  port3.pin1();

void resetServo()//Reset servo degree
{
  bottomArm1.write(resetDegree[0]);//0-180
  bottomArm2.write(resetDegree[1]);//40-70
  middleArm1.write(resetDegree[2]);//0-50
  middleArm2.write(resetDegree[3]);//10-30
  //topArm.write(resetDegree[4]);
}

void initServo()//Initial servo pin and reset
{
  bottomArm1.attach(servo1Pin);//底座旋转
  bottomArm2.attach(servo2Pin);//底座俯仰
  middleArm1.attach(servo3Pin);//中间上下
  middleArm2.attach(servo4Pin);//上部上下
  //topArm.attach(servo5Pin);
  resetServo();
}

//void openPaw(unsigned char motorSpeed)
//{
//  pawArm.run(- motorSpeed);
//  delay(1000);
//}
//
//void closePaw(unsigned char motorSpeed)
//{
//  pawArm.run(motorSpeed);
//  delay(1000);
//}

void getTrash(unsigned int degree,int distance)
{
  bottomArm1.write(degree);
  //shenshou
  
}

void go()
{ 
  if(ultraSensor1.distanceCm() < 15)
  {
    leftMotor.run(200);
    rightMotor.run(200);
  }
  else if(ultraSensor.distanceCm() < 15)
  {
    leftMotor.run(-200);
    rightMotor.run(-200);
  }
  else
  {
    leftMotor.run(100);
    rightMotor.run(-100);
  }
}


void servoAction(unsigned char idx,unsigned char action,unsigned char degree)//idx:伺服选择;action:0增加，1减小;degree:角度变化量
{
  if(action == 0)
  {
    currentDegree[idx] += degree;
  }
  else 
  {
    currentDegree[idx] -= degree;
  }
  if(idx == 3 && currentDegree[idx] > 30) currentDegree[idx] = 10;
  if(idx == 3 && currentDegree[idx] < 10) currentDegree[idx] = 30;
  if(idx == 2 && currentDegree[idx] < 0) currentDegree[idx] = 50;
  if(idx == 2 && currentDegree[idx] > 50) currentDegree[idx] = 0;
  if(idx == 1 && currentDegree[idx] < 40) currentDegree[idx] = 70;
  if(idx == 1 && currentDegree[idx] > 70) currentDegree[idx] = 40;
  if(idx == 0 && currentDegree[idx] < 10) currentDegree[idx] = 170;
  if(idx == 0 && currentDegree[idx] > 170) currentDegree[idx] = 10;
  switch(idx)
  {
    case 0:bottomArm1.write(currentDegree[idx]);break;
    case 1:bottomArm2.write(currentDegree[idx]);break;
    case 2:middleArm1.write(currentDegree[idx]);break;
    case 3:middleArm2.write(currentDegree[idx]);break;
    //default:topArm.write(currentDegree[idx]);break;
  }
  Serial.print("1:");// + currentDegree[0]);
  Serial.print(currentDegree[0]);
  Serial.print(" 2:");// + currentDegree[1]);
  Serial.print(currentDegree[1]);
  Serial.print(" 3:");// + currentDegree[2]);
  Serial.print(currentDegree[2]);
  Serial.print(" 4:");// + currentDegree[3]);
  Serial.print(currentDegree[3]);
  Serial.println("");
}

void action()
{
  uint8_t ReceiverCode;
  uint8_t buttonState;
  static uint8_t PrebuttonState = 0;

  buttonState = infraredReceiverDecode.buttonState();
  if(PrebuttonState != buttonState)
  {
    PrebuttonState = buttonState;
  }
  if(infraredReceiverDecode.available())
  {
    ReceiverCode = infraredReceiverDecode.read();
    switch(ReceiverCode)
    {
       case IR_BUTTON_A: servoAction(3,0,10); break;
       case IR_BUTTON_B: break;
       case IR_BUTTON_C: servoAction(3,1,10); break;
       case IR_BUTTON_D: servoAction(2,0,10); break;
       case IR_BUTTON_E: servoAction(2,1,10); break;
       case IR_BUTTON_F: servoAction(1,1,10); break;
       case IR_BUTTON_SETTING:state = 1 - state;delay(100); break;
       case IR_BUTTON_UP: leftMotor.run(100);rightMotor.run(-100); break;
       case IR_BUTTON_DOWN:leftMotor.run(-100);rightMotor.run(100); break;
       case IR_BUTTON_LEFT: break;
       case IR_BUTTON_RIGHT: break;
       case IR_BUTTON_0: servoAction(1,0,10); break;
       case IR_BUTTON_1: servoAction(0,1,10); break;
       case IR_BUTTON_2: leftMotor.run(0);rightMotor.run(0);delay(100);break;
       case IR_BUTTON_3: servoAction(0,0,10); break;
       case IR_BUTTON_4: pawArm.run(100); break;
       case IR_BUTTON_5: pawArm.run(0); break;
       case IR_BUTTON_6: pawArm.run(-100); break;
       case IR_BUTTON_7: break;
       case IR_BUTTON_8: break;
       case IR_BUTTON_9: break;
       default: break;
    }
  } 
  delay(10);
}

void action1()
{
  uint8_t ReceiverCode;
  uint8_t buttonState;
  static uint8_t PrebuttonState = 0;

  buttonState = infraredReceiverDecode.buttonState();
  if(PrebuttonState != buttonState)
  {
    PrebuttonState = buttonState;
  }
  if(infraredReceiverDecode.available())
  {
    ReceiverCode = infraredReceiverDecode.read();
    switch(ReceiverCode)
    {
       case IR_BUTTON_SETTING: state = 1 - state; break;
       default: break;
    }
  } 
  delay(10);
}

void setup() 
{ 
  initServo();
  Serial.begin(9600);
  infraredReceiverDecode.begin();
} 

void loop() 
{ 
  //bottomArm1.write(0);
  //go();
  if(state) go();
  else action();
} 
