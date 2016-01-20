/**
 * Robotic Arm for Makeblock
 * GDUT ROBOT TEAM
 * Copyright (C) 2015 - 2016  ChenZiping
 */

#include "MeOrion.h"
#include <Servo.h>
#include <SoftwareSerial.h>

#define SERVO_NUM 4
#define uc unsigned char
#define MOTOR_RIGHT(v) {leftMotor.run(v);rightMotor.run(v);}
#define MOTOR_LEFT(v) {leftMotor.run(-v);rightMotor.run(-v);}
#define MOTOR_STOP {leftMotor.run(0);rightMotor.run(0);}

//<模式选择
//#define RUN_MODE
#define DEBUG_MODE
//>模式选择

uc debugDegree_tmp[SERVO_NUM * 3] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uc debugDegree[SERVO_NUM] = {0, 0, 0, 0};
uc catchDegree[SERVO_NUM] = {90, 70, 50, 0};
uc resetDegree[SERVO_NUM] = {90, 50, 70, 66};
uc currentDegree[SERVO_NUM] = {90, 50, 70, 66};
uc leftLineData, rightLineData;

int i;
int state = 0;

MeLineFollower leftLine(PORT_3), rightLine(PORT_4);
MeDCMotor pawArm(PORT_1);
MeDCMotor leftMotor(M1), rightMotor(M2);
MePort port1(PORT_5),//1:底座旋转;2:底座俯仰
       port2(PORT_6);//1：中间上下;2:上部上下
Servo bottomArm1, bottomArm2, middleArm1, middleArm2;
MeUltrasonicSensor leftUltra(PORT_7);
MeUltrasonicSensor rightUltra(PORT_8);
MeInfraredReceiver infraredReceiverDecode(PORT_2);

int16_t servo1Pin =  port1.pin1();
int16_t servo2Pin =  port1.pin2();
int16_t servo3Pin =  port2.pin1();
int16_t servo4Pin =  port2.pin2();

void resetServo()//Reset servo degree
{
  bottomArm1.write(resetDegree[0]);//0-180 resetDegree[0]
  bottomArm2.write(resetDegree[1]);//40-70 1
  middleArm1.write(resetDegree[2]);//0-50
  middleArm2.write(resetDegree[3]);//10-30
  delay(100);
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

void go()//auto go avoid obstacle
{
  if(leftUltra.distanceCm() < 15)
  {
    leftMotor.run(200);
    rightMotor.run(200);
  }
  else if(rightUltra.distanceCm() < 15)
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
  if(action == 0) currentDegree[idx] += degree;
  else currentDegree[idx] -= degree;
  //<角度限制
  if(idx == 3 && currentDegree[idx] > 30) currentDegree[idx] = 10;
  if(idx == 3 && currentDegree[idx] < 10) currentDegree[idx] = 30;
  if(idx == 2 && currentDegree[idx] < 0) currentDegree[idx] = 50;
  if(idx == 2 && currentDegree[idx] > 50) currentDegree[idx] = 0;
  if(idx == 1 && currentDegree[idx] < 40) currentDegree[idx] = 70;
  if(idx == 1 && currentDegree[idx] > 70) currentDegree[idx] = 40;
  if(idx == 0 && currentDegree[idx] < 10) currentDegree[idx] = 170;
  if(idx == 0 && currentDegree[idx] > 170) currentDegree[idx] = 10;
  //>角度限制
  switch(idx)
  {
    case 0:bottomArm1.write(currentDegree[idx]);break;
    case 1:bottomArm2.write(currentDegree[idx]);break;
    case 2:middleArm1.write(currentDegree[idx]);break;
    case 3:middleArm2.write(currentDegree[idx]);break;
    //default:topArm.write(currentDegree[idx]);break;
  }
  //<实时角度显示
  Serial.print("1:");// + currentDegree[0]);
  Serial.print(currentDegree[0]);
  Serial.print(" 2:");// + currentDegree[1]);
  Serial.print(currentDegree[1]);
  Serial.print(" 3:");// + currentDegree[2]);
  Serial.print(currentDegree[2]);
  Serial.print(" 4:");// + currentDegree[3]);
  Serial.print(currentDegree[3]);
  Serial.println("");
  //>实时角度显示
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
       case IR_BUTTON_A: servoAction(3, 0, 10); break;
       case IR_BUTTON_B: break;
       case IR_BUTTON_C: servoAction(3, 1, 10); break;
       case IR_BUTTON_D: servoAction(2, 0, 10); break;
       case IR_BUTTON_E: servoAction(2, 1, 10); break;
       case IR_BUTTON_F: servoAction(1, 1, 10); break;
       case IR_BUTTON_SETTING:state = 1 - state;delay(100); break;
       case IR_BUTTON_UP: leftMotor.run(100);rightMotor.run(-100); break;
       case IR_BUTTON_DOWN:leftMotor.run(-100);rightMotor.run(100); break;
       case IR_BUTTON_LEFT: break;
       case IR_BUTTON_RIGHT: break;
       case IR_BUTTON_0: servoAction(1, 0, 10); break;
       case IR_BUTTON_1: servoAction(0, 1, 10); break;
       case IR_BUTTON_2: leftMotor.run(0);rightMotor.run(0);delay(100);break;
       case IR_BUTTON_3: servoAction(0, 0, 10); break;
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

//串口多舵机实时串口调试工具 by 陈子平 & 张诚钊
void servoSerialDebuger()
{
  if(Serial.available()) //如果有数据输入
  {
    delay(120); //等待所有输入数据传输完毕
    if(Serial.available() == SERVO_NUM * 3) //传入4个3位角度值
    {
      while (Serial.available()) //一位位读取数据
      {
        debugDegree_tmp[i++] = Serial.read() - 48; //读到的数据是ASCII码，减去48（‘0’的ASCII码）得到输入数字
      }
      //<计算出要调整的角度数据
      for(i = 0;i < SERVO_NUM * 3;i ++) debugDegree[i] = debugDegree_tmp[i] * 100 + debugDegree_tmp[++i] * 10 + debugDegree_tmp[++i];
      //>计算出要调整的角度数据
      //<控制舵机转动
      bottomArm1.write(debugDegree[0]);
      bottomArm2.write(debugDegree[1]);
      middleArm1.write(debugDegree[2]);
      middleArm2.write(debugDegree[3]);
      //>控制舵机转动
      //<实时显示更新角度
      Serial.println("1\t2\t3\t4");
      for(i = 0;i < SERVO_NUM;i ++)
      {
        Serial.print(debugDegree[i]);
        Serial.print("\t");
      }
      Serial.println("");
      //实时显示更新角度>
      delay(8000);
      //<初始化i
      i = 0;
      //>初始化i
    }
    else Serial.flush(); //刷新串口输入缓存
  }
}

void lineFlower()//read lineflower data
{
  leftLineData = leftLine.readSensors();
  rightLineData = rightLine.readSensors();
}

void pickup()
{
  bottomArm1.write(catchDegree[0]);
  bottomArm2.write(catchDegree[1]);
  middleArm1.write(catchDegree[2]);
  middleArm2.write(catchDegree[3]);
}

void gogogo()
{
  if(leftUltra.distanceCm() < 20 || rightUltra.distanceCm() < 20)
  {
    MOTOR_STOP;
    pickup();
    delay(1000);
  }
  else
  {
    if(leftLineData) MOTOR_RIGHT(50)
    else if(rightLineData) MOTOR_RIGHT(50)
  }
}

void setup()//initial
{
  initServo();
  Serial.begin(9600);
  infraredReceiverDecode.begin();
}

void loop()//loop
{
  /*if(state) go();
  else action();*/
  #ifdef RUN_MODE
  resetServo();//reset servo degree
  lineFlower();
  gogogo();
  #elif define DEBUG_MODE
  servoSerialDebuger();
  #endif
//  Serial.println(Serial.read());
//  Serial.println(LF1);
//  Serial.println(LF2);
}
