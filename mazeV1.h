#ifndef MAZEV1_H_
#define MAZEV1_H_

#include <Arduino.h>
#include <GyverOLED.h>
#include <analogWrite.h>

#define mazeInit mazeInit
void mazeInit();

#define led 12
// Multiplexer
#define sel0 23
#define sel1 5
#define sel2 19
#define sel3 18
#define EN_F 13
#define EN_R 4
// Adc
#define ADC 2
// motor
#define motorKn1 27
#define motorKn2 26
#define motorKr1 25
#define motorKr2 33
// button
#define btn1 35
// logic
#define ff true
#define bb false
// encoder
#define encKn 15
#define encKr 32
#define pid1() pidvalue(0.04, 0.00, 1.25)  // speed 100
#define pid2() pidvalue(0.045, 0.00, 1.75) // pid sik gendeng
// end pid
void pidvalue(float kp_, float ki_, float kd_);
// function
int errorlineF(bool kondisi);
float errorF(bool kondisi);
int btnT();
void readmuxF();
void readmuxB();
int errorlineF(bool kondisi);
bool detectcross(bool sensor);
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
void linefollower(int Skiri, int Skanan, bool sensor);
void linecrossfind(int Skiri, int Skanan, bool sensor, int rem);
void noLinefind(int Skiri, int Skanan, bool sensor);
void bkanan(int speed, bool sensor, int rem);
void bkiri(int speed, bool sensor, int rem);
void majuremS(int timer_);
void mundurremS(int timer_);
void IRAM_ATTR pulseCountKn();
void IRAM_ATTR pulseCountKr();
int pulsaKn();
int pulsaKr();
float mmKn();
float mmKr();
void majuenC(int speed, bool arah, int jkanan, int jkiri, int brake);

#endif