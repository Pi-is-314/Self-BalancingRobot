// MultiStepper.pde
// -*- mode: C++ -*-
//
// Shows how to multiple simultaneous steppers
// Runs one stepper forwards and backwards, accelerating and decelerating
// at the limits. Runs other steppers at the same time
//
// Copyright (C) 2009 Mike McCauley
// $Id: MultiStepper.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

 
/*
  MPU6050 Raw

  A code for obtaining raw data from the MPU6050 module with the option to
  modify the data output format.

  Find the full MPU6050 library documentation here:
  https://github.com/ElectronicCats/mpu6050/wiki
*/

//Libraries
#include <AccelStepper.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <SPI.h>
#include <MD_MAX72xx.h> 
#include <BLEGamepadClient.h>
#include <StorageExpressions.cpp>


XboxController controller;


#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define CS_PIN 5


//Code for the intializing the stepper motors
//STEPPER 1 PINS
const int step1Pin = 26;
const int dir1Pin = 25;

//STEPPER 2 PINS
const int step2Pin = 17;
const int dir2Pin = 16;

//DISPLAY PINS PINS
const int DIN = 23;
const int CS = 5;
const int CLK = 18;
 
//CONSTANTS FOR PID
const float kP = 10;
const float kI = 0.1;
const float kD = 0.1;

const double idealAngle = 0.0;
const double maxMotorSpeed = 400.0;
const double motorAcceleration = 150.0;
const double deadband = 0.5;
const double maxIntegral = 100.0;
double currentAngle = 0.0;
double totalError = 0.0;
double previousError = 0.0;
double filteredSpeed = 0.0;

//Defines the displays
MD_MAX72XX matrix(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

AccelStepper stepper1(AccelStepper::DRIVER, step1Pin, dir1Pin);
AccelStepper stepper2(AccelStepper::DRIVER, step2Pin, dir2Pin);

// MPU Constants
MPU6050 mpu(Wire); //SDA = PIN 21, SCL = PIN 22
unsigned long timer = 0;

//Variables for the Eye Shape
bool winking = false;

//METHODS DEFINITION
double calculatePID(); 
void runExpressions(int (&matrix)[8][8]);
void setBothSpeed(double speed);
void handleXboxControllerEvents();

void setup() {
  matrix.begin();
  
  Serial.begin(115200);
  Serial.print("hello");
  controller.begin();
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  stepper1.setMaxSpeed(maxMotorSpeed);
  stepper1.setAcceleration(motorAcceleration);
  stepper1.setSpeed(0);

  stepper2.setMaxSpeed(maxMotorSpeed);
  stepper2.setAcceleration(motorAcceleration);
  stepper2.setSpeed(0);
}

void loop() {
  mpu.update();
  currentAngle = mpu.getAngleY();
  if((millis()-timer)>10){ // print data every 10ms
	Serial.print("X : ");
	Serial.print(mpu.getAngleX());
	Serial.print("\tY : ");
	Serial.print(mpu.getAngleY());
	Serial.print("\tZ : ");
	Serial.println(mpu.getAngleZ());
	timer = millis();  
  }
  double rawSpeed = calculatePID();

  setBothSpeed(rawSpeed);
  handleXboxControllerEvents();
  
}

double calculatePID() {
  double error = idealAngle - currentAngle;
  totalError += error;
  totalError = constrain(totalError, -maxIntegral, maxIntegral);
  double derivative = error - previousError;
  double output = kP * error + kI * totalError + kD * derivative;
  previousError = error;
  return output;
}

void setBothSpeed(double speed){
  stepper1.setSpeed(speed);
  stepper2.setSpeed(speed);
  stepper1.runSpeed();
  stepper2.runSpeed();
}

void runLeftExpressions(int (&matrixData)[8][8]){
  for(int i = 0; i < 8; i++){
    for(int j = 0; j < 8; j++){
      if(matrixData[i][j] == 1){
        matrix.setPoint(i, j, true);
      }
      else{
        matrix.setPoint(i, j, false);

      }
    }
  }
}


void runRightExpressions(int (&matrixData)[8][8]){
  for(int i = 0; i < 8; i++){
    for(int j = 8; j < 16; j++){
      if(matrixData[i][j] == 1){
        matrix.setPoint(i, j, true);
      }
      else{
        matrix.setPoint(i, j, false);

      }
    }
  }
}
void handleXboxControllerEvents() {
  if (controller.isConnected()) {
      XboxControlsState s;
      controller.read(&s);

      Serial.printf("lstick: %.2f,%.2f, rstick: %.2f,%.2f\n",
        s.leftStickX, s.leftStickY, s.rightStickX, s.rightStickY);
      if (s.buttonA){
        runLeftExpressions(normalFace);
        runRightExpressions(normalFace);
      }
      else if (s.buttonB){
        runLeftExpressions(LeftWinking);
        runRightExpressions(RightWinking);
      }
      else{
        matrix.clear();
      }
    } else {
      Serial.println("controller not connected");
    }
}