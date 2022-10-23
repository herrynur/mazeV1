#ifndef MAZEV1_H_
#define MAZEV1_H_

#include <Arduino.h>
#include <GyverOLED.h>
#include <analogWrite.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define mazeInit mazeInit
void mazeInit();

#define led 12
// Multiplexer
#define sel0 19
#define sel1 18
#define sel2 5
#define sel3 23
#define EN_F 13
#define EN_R 33
// Adc
#define ADC 14
// motor
#define motorKn1 25
#define motorKn2 4
#define motorKr1 27
#define motorKr2 26
// button
#define btn1 35
#define btn2 34
#define btn3 36
#define btn4 39
// logic
#define ff true
#define bb false
//logic belok
#define kiri true
#define kanan false
//logic warna
#define putih false
#define hitam true
//logic maju mundur
#define _maju true
#define _mundur false
// encoder
#define encKn 15
#define encKr 32
#define pid1() pidvalue(0.04, 0.00, 1.25)  // speed 100
#define pid2() pidvalue(0.045, 0.00, 1.75) // pid sik gendeng
// end pid
void pidvalue(float kp_, float ki_, float kd_);
// function
int errorlineB(bool kondisi);
float errorF(bool kondisi);
int btnT();
void readmuxF();
void readmuxB();

int errorlineB(bool kondisi);
int errorlineW(bool kondisi);

float newErrorlineB(bool kondisi);
float newErrorlineW(bool kondisi);

bool detectcross(bool sensor, bool warna);
bool detectCenter(bool sensor, bool warna);
float errorF(bool kondisi);
void mundur(int speed_);
void maju(int speed_);
void mtrknMj(int speed_);
void mtrkrMj(int speed_);
void mtrknMn(int speed_);
void mtrkrMn(int speed_);
void belokiri(int speed_);
void belokanan(int speed_);
void pkiri(int speed_);
void pkanan(int speed_);
void majuspeed(int Skanan, int Skiri);
void majuzero(int Skanan, int Skiri);
void mundurspeed(int Skanan, int Skiri);
void pkiriT(int speed_, int timer);
void pkananT(int speed_, int timer);
void majutimer(int Skanan, int Skiri, int timer);
void mundurtimer(int Skanan, int Skiri, int timer);
//line follower
void linefollower(int Skiri, int Skanan, bool sensor, bool warna);
void lfEncoder(int Skiri, int Skanan, bool sensor, int jarak, int rem, bool warna);
void linecrossfind(int Skiri, int Skanan, bool sensor, int rem, bool warna);
void noLinefind(int Skiri, int Skanan, bool sensor, bool warna, int rem);
void findCross(int speed, bool sensor, bool warna, int rem);
void lfDelay(int Skiri, int Skanan, bool sensor, bool warna, int rem, int delay_);
int errorKhusus(bool kondisi);

void bkanan(int speed, bool sensor, int rem, bool warna);
void bkiri(int speed, bool sensor, int rem, bool warna);
void majuremS(int timer_);
void mundurremS(int timer_);
bool _button(int pin);
void IRAM_ATTR pulseCountKn();
void IRAM_ATTR pulseCountKr();
int pulsaKn();
int pulsaKr();
float mmKn();
float mmKr();
void motorEnc(int speed, bool arah, int jkanan, int jkiri, int brake);
void belokEnc(int speed, bool arahBelok, int jaraKiri, int jaraKanan);
void motorBerhenti();

void kalibrasi();

//pid
void pidvalue(float kp_, float ki_, float kd_);

//Servo
#define buka true
#define tutup false

void servoCapit(bool kondisi);

#define pickup true
#define pickdown false

void servoPickup();
void servoPickDown();

void capitOpen();
void capitClose();

void lempar();
#endif