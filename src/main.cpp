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
#include "I2Cdev.h"
#include <AccelStepper.h>
#include <Wire.h>
#include <MPU6050_light.h>


//Code for the intializing the stepper motors
const int step1Pin = 26;
const int dir1Pin = 25;

const int step2Pin = 19;
const int dir2Pin = 18;

AccelStepper stepper1(AccelStepper::DRIVER, step1Pin, dir1Pin);
AccelStepper stepper2(AccelStepper::DRIVER, step2Pin, dir2Pin);



//MPU Constants
MPU6050 mpu(Wire); //SDA = PIN 21, SCL = PIN 22
unsigned long timer = 0;

void setup() {
  Serial.begin(9600);
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

  stepper1.setMaxSpeed(800);
  stepper1.setAcceleration(400);
  stepper1.moveTo(200);

  stepper2.setMaxSpeed(800);
  stepper2.setAcceleration(400);
  stepper2.moveTo(200);
}

void loop() {
  mpu.update();
  
  if((millis()-timer)>10){ // print data every 10ms
	Serial.print("X : ");
	Serial.print(mpu.getAngleX());
	Serial.print("\tY : ");
	Serial.print(mpu.getAngleY());
	Serial.print("\tZ : ");
	Serial.println(mpu.getAngleZ());
	timer = millis();  
  }
  // if (stepper.distanceToGo() == 0) {
  //   stepper.moveTo(-stepper.currentPosition());
  // }
  stepper1.run();
  stepper2.run();
}

