#include "mazeV1.h"
// oled
GyverOLED<SSH1106_128x64> oled;
// button
int btnC = 0;
int btnT();
int Mc = 0;
int btnS = 1;
// front
int hasil_adc[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int s[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int caseSensor;
// var
float errF = 0;
float errB = 0;
// encoder
volatile uint32_t counterKn = 0;
volatile uint32_t counterKr = 0;
unsigned long lastTimeKn = 0;
unsigned long lastTimeKr = 0;
// pid
// float KP = 0.038, KI = 0.0, KD = 3.00;
float kp = 0.00;
float ki = 0.0;
float kd = 0.00;

void setup()
{
    Serial.begin(115200);
    oled.init();
    // pinmode
    pinMode(led, OUTPUT);
    // adc
    pinMode(ADC, INPUT);
    // multiplexer
    pinMode(sel0, OUTPUT);
    pinMode(sel1, OUTPUT);
    pinMode(sel2, OUTPUT);
    pinMode(sel3, OUTPUT);
    pinMode(EN_F, OUTPUT);
    pinMode(EN_R, OUTPUT);
    pinMode(ADC, INPUT);
    pinMode(led, OUTPUT);

    digitalWrite(sel0, LOW);
    digitalWrite(sel1, LOW);
    digitalWrite(sel2, LOW);
    digitalWrite(sel3, LOW);
    digitalWrite(EN_F, HIGH);
    digitalWrite(EN_R, HIGH);
    digitalWrite(led, LOW);
    // button
    pinMode(btn1, INPUT_PULLUP);
    // motor
    pinMode(motorKn1, OUTPUT);
    pinMode(motorKn2, OUTPUT);
    pinMode(motorKr1, OUTPUT);
    pinMode(motorKr2, OUTPUT);
    digitalWrite(motorKn1, LOW);
    digitalWrite(motorKn1, LOW);
    digitalWrite(motorKn1, LOW);
    digitalWrite(motorKn1, LOW);
    // oled
    oled.update();
    delay(2);
    oled.clear();
    oled.setScale(2);
    oled.setCursorXY(0, 5);
    oled.print(" MAZE V.1");
    delay(1000);
    oled.clear();
    oled.update();
    // encoder
    pinMode(encKn, INPUT_PULLUP);
    pinMode(encKr, INPUT_PULLUP);
    pinMode(led, OUTPUT);
    // custom init
    mazeInit();
}

// <---- Fungsi Pembacaan Sensor disini --->
// --------------------------------------------------

// <---- Akhir Fungsi Pembacaan Sensor --->
// --------------------------------------------------
void readmuxF()
{
  digitalWrite(EN_F, 0);
  digitalWrite(EN_R, 1);
  // 1101
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 1);
  s[0] = analogRead(ADC);
  // 0101
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 1);
  s[1] = analogRead(ADC);
  // 1001
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 1);
  s[2] = analogRead(ADC);
  // 0001
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 1);
  s[3] = analogRead(ADC);
  // 0000
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 0);
  s[4] = analogRead(ADC);
  // 1000
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 0);
  s[5] = analogRead(ADC);
  // 0100
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 0);
  s[6] = analogRead(ADC);
  // 1110
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 1);
  digitalWrite(sel3, 0);
  s[7] = analogRead(ADC);
  // 0110
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 1);
  digitalWrite(sel3, 0);
  s[8] = analogRead(ADC);
  // 1010
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 1);
  digitalWrite(sel3, 0);
  s[9] = analogRead(ADC);
  // 0010
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 1);
  digitalWrite(sel3, 0);
  s[10] = analogRead(ADC);
  // 1100
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 0);
  s[11] = analogRead(ADC);
  ////
  // for (int i = 0; i <= 11; i++)
  // {
  //   Serial.print(s[i]);
  //   Serial.print("-");
  // }
  // Serial.println();
  /////////////////////////////////////
  for (int i = 0; i <= 11; i++)
  {
    if (s[i] >= 3500)
    {
      hasil_adc[i] = 1;
    }
    else
    {
      hasil_adc[i] = 0;
    }
  }
  // oled.setScale(1);
  // oled.setCursorXY(0, 4);
  // oled.print("F = ");
  for (int i = 0; i <= 11; i++)
  {
    oled.print(hasil_adc[i]);
    Serial.print(hasil_adc[i]);
    Serial.print("-");
  }
  Serial.println();

  caseSensor = ((hasil_adc[11] * 0) + (hasil_adc[10] * 1) + (hasil_adc[9] * 2) + (hasil_adc[8] * 4) + (hasil_adc[7] * 8) + (hasil_adc[6] * 16) + (hasil_adc[5] * 32) + (hasil_adc[4] * 64) + (hasil_adc[3] * 128) + (hasil_adc[2] * 256) + (hasil_adc[1] * 512) + (hasil_adc[0] * 1024));
}

void readmuxB()
{
  digitalWrite(EN_F, 1);
  digitalWrite(EN_R, 0);
  // 1101
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 1);
  s[0] = analogRead(ADC);
  // 0101
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 1);
  s[1] = analogRead(ADC);
  // 1001
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 1);
  s[2] = analogRead(ADC);
  // 0001
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 1);
  s[3] = analogRead(ADC);
  // 0000
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 0);
  s[4] = analogRead(ADC);
  // 1000
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 0);
  s[5] = analogRead(ADC);
  // 0100
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 0);
  s[6] = analogRead(ADC);
  // 1110
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 1);
  digitalWrite(sel3, 0);
  s[7] = analogRead(ADC);
  // 0110
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 1);
  digitalWrite(sel3, 0);
  s[8] = analogRead(ADC);
  // 1010
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 1);
  digitalWrite(sel3, 0);
  s[9] = analogRead(ADC);
  // 0010
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 1);
  digitalWrite(sel3, 0);
  s[10] = analogRead(ADC);
  // 1100
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 0);
  s[11] = analogRead(ADC);
  ///////
  for (int i = 11; i >= 0; i--)
  {
    Serial.print(s[i]);
    Serial.print("-");
  }
  Serial.println();
  /////////////////////////////////////
  for (int i = 0; i <= 11; i++)
  {
    if (s[i] >= 4000)
    {
      hasil_adc[i] = 1;
    }
    else
    {
      hasil_adc[i] = 0;
    }
  }
  // oled.setScale(1);
  // oled.setCursorXY(0, 16);
  // oled.print("R = ");
  // for (int i = 11; i >= 0; i--)
  // {
  //   oled.print(hasil_adc[i]);
  // }
  // Serial.println();
  // oled.update();
}

int errorlineF(bool kondisi)
{
  int errorline = 0;
  if (kondisi == ff)
    errorline = errorF(ff);
  if (kondisi == bb)
    errorline = errorF(bb);
  int16_t error_ = 0;
  switch (errorline)
  {
  case 0b111110000000:
    error_ = -90;
    break;
  case 0b111100000000:
    error_ = -100;
    break;
  case 0b100000000000:
    error_ = -76;
    break;
  case 0b110000000000:
    error_ = -60;
    break;
  case 0b010000000000:
    error_ = -46;
    break;
  case 0b011000000000:
    error_ = -34;
    break;
  case 0b001000000000:
    error_ = -24;
    break;
  case 0b001100000000:
    error_ = -16;
    break;
  case 0b000100000000:
    error_ = -10;
    break;
  case 0b000110000000:
    error_ = -6;
    break;
  case 0b000010000000:
    error_ = -4;
    break;
  case 0b000011000000:
    error_ = -2;
    break;
  case 0b000001100000:
    error_ = 0;
    break;
  case 0b000000100000:
    error_ = 0;
    break;
  case 0b000001000000:
    error_ = 0;
    break;
  case 0b000001110000:
    error_ = 1;
    break;
  case 0b000000110000:
    error_ = 2;
    break;
  case 0b000000010000:
    error_ = 4;
    break;
  case 0b000000011000:
    error_ = 6;
    break;
  case 0b000000001000:
    error_ = 10;
    break;
  case 0b000000001100:
    error_ = 16;
    break;
  case 0b000000000100:
    error_ = 24;
    break;
  case 0b000000000110:
    error_ = 34;
    break;
  case 0b000000000010:
    error_ = 46;
    break;
  case 0b000000000011:
    error_ = 60;
    break;
  case 0b000000000001:
    error_ = 76;
    break;
  case 0b000000001111:
    error_ = 100;
    break;
  case 0b000000011111:
    error_ = 90;
    break;
    // lostline
  case 0b000000000000:
    error_ = 100;
    break;
    // crossfind
  case 0b111111111111:
    error_ = 200;
    break;
  case 0b000001111111:
    error_ = 50;
    break;
  case 0b111111000000:
    error_ = -50;
    break;
  }
  return error_;
}

bool detectcross(bool sensor)
{
  int s[2];
  bool hasil;
  if (sensor == ff)
  {
    digitalWrite(EN_F, 0);
    digitalWrite(EN_R, 1);
  }
  if (sensor == bb)
  {
    digitalWrite(EN_F, 1);
    digitalWrite(EN_R, 0);
  }
  // Sensor paling kiri
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 1);
  s[0] = analogRead(ADC);
  // Sensor paling kanan
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 0);
  s[1] = analogRead(ADC);
  if (s[0] > 3500 && s[1] > 3500)
  {
    hasil = true;
  }
  else
  {
    hasil = false;
  }
  return hasil;
}

float errorF(bool kondisi)
{
  if (kondisi == ff)
  {
    digitalWrite(EN_F, 0);
    digitalWrite(EN_R, 1);
  }
  if (kondisi == bb)
  {
    digitalWrite(EN_F, 1);
    digitalWrite(EN_R, 0);
  }
  // 1101
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 1);
  s[0] = analogRead(ADC);
  // 0101
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 1);
  s[1] = analogRead(ADC);
  // 1001
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 1);
  s[2] = analogRead(ADC);
  // 0001
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 1);
  s[3] = analogRead(ADC);
  // 0000
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 0);
  s[4] = analogRead(ADC);
  // 1000
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 0);
  s[5] = analogRead(ADC);
  // 0100
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 0);
  s[6] = analogRead(ADC);
  // 1110
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 1);
  digitalWrite(sel3, 0);
  s[7] = analogRead(ADC);
  // 0110
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 1);
  digitalWrite(sel3, 0);
  s[8] = analogRead(ADC);
  // 1010
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 1);
  digitalWrite(sel3, 0);
  s[9] = analogRead(ADC);
  // 0010
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 1);
  digitalWrite(sel3, 0);
  s[10] = analogRead(ADC);
  // 1100
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 0);
  s[11] = analogRead(ADC);

  for (int i = 0; i <= 11; i++)
  {
    if (s[i] >= 3700)
    {
      hasil_adc[i] = 1;
    }
    else
    {
      hasil_adc[i] = 0;
    }
  }
  caseSensor = ((hasil_adc[11] * 1) + (hasil_adc[10] * 2) + (hasil_adc[9] * 4) + (hasil_adc[8] * 8) + (hasil_adc[7] * 16) + (hasil_adc[6] * 32) + (hasil_adc[5] * 64) + (hasil_adc[4] * 128) + (hasil_adc[3] * 256) + (hasil_adc[2] * 512) + (hasil_adc[1] * 1024) + (hasil_adc[0] * 2048));
  return caseSensor;
}
// <---- Fungsi Encoder motor disini --->
// ----------------------------------------

void IRAM_ATTR pulseCountKn()
{
    digitalWrite(led, !digitalRead(led));
    counterKn++;
}

void IRAM_ATTR pulseCountKr()
{
    digitalWrite(led, !digitalRead(led));
    counterKr++;
}

int pulsaKn()
{
    detachInterrupt(encKn);
    unsigned long dtime = ((unsigned long)millis() - lastTimeKn);
    float rpm = counterKn;
    lastTimeKn = millis();
    attachInterrupt(encKn, pulseCountKn, RISING);
    return rpm;
}

int pulsaKr()
{
    detachInterrupt(encKr);
    unsigned long dtime = ((unsigned long)millis() - lastTimeKr);
    float rpm = counterKr;
    lastTimeKr = millis();
    attachInterrupt(encKr, pulseCountKr, RISING);
    return rpm;
}

float mmKn()
{
    // diameter 141 mm
    // 0.6714
    float pulse = pulsaKn();
    float mm = pulse * 0.6714;
    return mm;
}

float mmKr()
{
    // diameter 141 mm
    // 0.6714
    float pulse = pulsaKr();
    float mm = pulse * 0.6714;
    return mm;
}

void majuenC(int speed, bool arah, int jkanan, int jkiri, int brake)
{
    bool isComplete = false;
    bool Ckiri = false;
    bool Ckanan = false;
    float kiri = 0;
    float kanan = 0;
    while (!isComplete)
    {
        kiri = mmKr();
        kanan = mmKn();
        Serial.print(kiri);
        Serial.print(" - ");
        Serial.println(kanan);
        if (arah == ff)
        {
            mtrknMj(speed);
            mtrkrMj(speed);
        }
        if (arah == bb)
        {
            mtrknMn(speed);
            mtrkrMn(speed);
        }
        if (kiri >= jkiri)
        {
            if (arah == ff)
                mtrkrMj(0);
            if (arah == bb)
                mtrkrMn(0);
            Ckiri = true;
        }
        if (kanan >= jkanan)
        {
            if (arah == ff)
                mtrknMj(0);
            if (arah == bb)
                mtrknMn(0);
            Ckanan = true;
        }
        if (Ckiri == true && Ckanan == true)
            isComplete = true;
    }
    if (arah == ff)
        mundurremS(brake);
    if (arah == bb)
        majuremS(brake);
    counterKn = 0;
    counterKr = 0;
    lastTimeKn = 0;
    lastTimeKr = 0;
}
// <---- Akhir Fungsi Encoder motor disini --->
// -------------------------------------------


// <---- Fungsi kinematik motor disini --->
// ----------------------------------------
void mundur(int speed_)
{
  analogWrite(motorKn1, speed_);
  analogWrite(motorKn2, 0);
  analogWrite(motorKr1, speed_);
  analogWrite(motorKr2, 0);
}

void maju(int speed_)
{
  analogWrite(motorKn1, 0);
  analogWrite(motorKn2, speed_);
  analogWrite(motorKr1, 0);
  analogWrite(motorKr2, speed_);
}

void mtrknMj(int speed_)
{
  analogWrite(motorKn1, 0);
  analogWrite(motorKn2, speed_);
}

void mtrkrMj(int speed_)
{
  analogWrite(motorKr1, 0);
  analogWrite(motorKr2, speed_);
}

void mtrknMn(int speed_)
{
  analogWrite(motorKn1, speed_);
  analogWrite(motorKn2, 0);
}

void mtrkrMn(int speed_)
{
  analogWrite(motorKr1, speed_);
  analogWrite(motorKr2, 0);
}

void belokiri(int speed_)
{
  mtrknMj(speed_);
  mtrkrMj(speed_ / 2);
}

void belokanan(int speed_)
{
  mtrknMj(speed_ / 2);
  mtrkrMj(speed_);
}

void pkiri(int speed_)
{
  mtrknMj(speed_);
  mtrkrMn(speed_);
}

void pkanan(int speed_)
{
  mtrknMn(speed_);
  mtrkrMj(speed_);
}

void majuspeed(int Skanan, int Skiri)
{
  if (Skanan > 0)
    mtrknMj(Skanan);
  if (Skiri > 0)
    mtrkrMj(Skiri);
  if (Skanan < 0)
    mtrknMn(Skanan * -1);
  if (Skiri < 0)
    mtrkrMn(Skiri * -1);
  // if (Skanan == 0)
  //   mtrknMn(Skanan);
  // if (Skiri == 0)
  //   mtrkrMn(Skiri);
}

void majuzero(int Skanan, int Skiri)
{
  if (Skanan > 0)
    mtrknMj(Skanan);
  if (Skiri > 0)
    mtrkrMj(Skiri);
  if (Skanan < 0)
    mtrknMn(Skanan * -1);
  if (Skiri < 0)
    mtrkrMn(Skiri * -1);
  if (Skanan == 0)
    mtrknMn(Skanan);
  if (Skiri == 0)
    mtrkrMn(Skiri);
}

void mundurspeed(int Skanan, int Skiri)
{
  mtrknMn(Skanan);
  mtrkrMn(Skiri);
}

void majuremS(int timer_)
{
  bool isTimeout = false;
  unsigned long timeStart = millis();
  while (!isTimeout)
  {
    majuzero(125, 125);
    if ((unsigned long)millis() - timeStart > timer_)
    {
      majuzero(0, 0);
      isTimeout = true;
    }
  }
}

void mundurremS(int timer_)
{
  bool isTimeout = false;
  unsigned long timeStart = millis();
  while (!isTimeout)
  {
    majuzero(-125, -125);
    if ((unsigned long)millis() - timeStart > timer_)
    {
      majuzero(0, 0);
      isTimeout = true;
    }
  }
}

void pkiriT(int speed_, int timer)
{
  bool isTimeout = false;
  unsigned long timeStart = millis();
  while (!isTimeout)
  {
    mtrknMj(speed_);
    mtrkrMn(speed_);
    if ((unsigned long)millis() - timeStart > timer)
    {
      isTimeout = true;
      majuspeed(0, 0);
    }
  }
}

void pkananT(int speed_, int timer)
{
  bool isTimeout = false;
  unsigned long timeStart = millis();
  while (!isTimeout)
  {
    mtrknMn(speed_);
    mtrkrMj(speed_);
    if ((unsigned long)millis() - timeStart > timer)
    {
      isTimeout = true;
      majuspeed(0, 0);
    }
  }
}

void majutimer(int Skanan, int Skiri, int timer)
{
  bool isTimeout = false;
  unsigned long timeStart = millis();
  while (!isTimeout)
  {
    majuzero(Skanan, Skiri);
    if ((unsigned long)millis() - timeStart > timer)
    {
      majuzero(0, 0);
      isTimeout = true;
    }
  }
}

void mundurtimer(int Skanan, int Skiri, int timer)
{
  bool isTimeout = false;
  unsigned long timeStart = millis();
  while (!isTimeout)
  {
    majuzero(Skanan, Skiri);
    if ((unsigned long)millis() - timeStart > timer)
    {
      majuzero(0, 0);
      isTimeout = true;
    }
  }
}
// <---- Akhir Fungsi kinematik motor disini --->
// ---------------------------------------------

// <---- Fungsi Line Follower motor disini --->
// --------------------------------------------
void linefollower(int Skiri, int Skanan, bool sensor)
{
  float errorB, error, lasterror = 0, sumerror = 0;
  // float KP = 0.038, KI = 0.0, KD = 3.00;
  float KP = kp, KI = ki, KD = kd;
  float P, I, D, out;
  float speedKa, speedKi;

  while (true)
  {
    if (sensor == ff)
      error = errorlineF(ff);
    if (sensor == bb)
      error = errorlineF(bb) * -1;

    Serial.println(error);

    errorB = error;
    if (error != 0)
      sumerror += error;
    else
      sumerror = 0;

    P = error * KP;
    D = (error - lasterror) * KD;
    I = sumerror * KI;
    out = P + I + D;

    error = lasterror;

    speedKi = Skiri + out;  // speed motor kiri
    speedKa = Skanan - out; // speed motor kanan

    if (speedKi >= 255)
      speedKi = 255;
    if (speedKa >= 255)
      speedKa = 255;
    if (speedKi <= -255)
      speedKi = -255;
    if (speedKa <= -255)
      speedKa = -255;

    // motor jalan
    if (sensor == ff)
    {
      if (errorB == 100 || errorB == -100)
        majuspeed(0, 0);
      else
        majuspeed(speedKa, speedKi);
    }

    else if (sensor == bb)
    {
      if (errorB == 100 || errorB == -100)
        mundurspeed(0, 0);
      else
        mundurspeed(speedKa, speedKi);
    }

    else
    {
      Serial.println("Else");
    }

    // tampil oled
    //  oled.setScale(1);
    //  oled.setCursorXY(0, 40);
    //  oled.print("out = ");
    //  oled.print(speedKi);
    //  oled.print("  ");
    //  oled.print(speedKa);
    //  oled.print("                              ");
    //  oled.update();
  }
}

void linecrossfind(int Skiri, int Skanan, bool sensor, int rem)
{
  float errorBf, error, lasterror = 0, sumerror = 0;
  float KP = kp, KI = ki, KD = kd;
  float P, I, D, out;
  float speedKa, speedKi;
  bool errorB;

  bool isFind = false;
  while (!isFind)
  {
    if (sensor == ff)
    {
      error = errorlineF(ff);
      errorB = detectcross(ff);
    }

    if (sensor == bb)
    {
      error = errorlineF(bb) * -1;
      errorB = detectcross(bb);
    }
    errorBf = error;

    if (errorB == true)
      isFind = true;

    if (error != 0)
      sumerror += error;
    else
      sumerror = 0;

    P = error * KP;
    D = (error - lasterror) * KD;
    I = sumerror * KI;
    out = P + I + D;

    error = lasterror;

    speedKi = Skiri + out;
    speedKa = Skanan - out;

    if (speedKi >= 255)
      speedKi = 255;
    if (speedKa >= 255)
      speedKa = 255;
    if (speedKi <= -255)
      speedKi = -255;
    if (speedKa <= -255)
      speedKa = -255;

    // motor jalan
    if (sensor == ff)
    {
      if (errorBf == 100 || errorBf == -100)
        majuspeed(0, 0);
      else
        majuspeed(speedKa, speedKi);
    }

    else if (sensor == bb)
    {
      if (errorBf == 100 || errorBf == -100)
        majuspeed(0, 0);
      else
        majuspeed(speedKa * -1, speedKi * -1);
    }

    else
    {
      Serial.println("Else");
    }
  }
  if (rem < 0)
    mundurtimer(Skanan, Skiri, rem * -1);
  if (rem > 0)
    majutimer(Skanan, Skiri, rem);
}

void noLinefind(int Skiri, int Skanan, bool sensor)
{
  float errorB, error, lasterror = 0, sumerror = 0;
  float KP = kp, KI = ki, KD = kd;
  float P, I, D, out;
  float speedKa, speedKi;

  bool isFind = false;
  while (!isFind)
  {
    if (sensor == ff)
      error = errorlineF(ff);
    if (sensor == bb)
      error = errorlineF(bb) * -1;

    errorB = error;
    if (errorB == 100 || errorB == -100)
      isFind = true;

    if (error != 0)
      sumerror += error;
    else
      sumerror = 0;

    P = error * KP;
    D = (error - lasterror) * KD;
    I = sumerror * KI;
    out = P + I + D;

    error = lasterror;

    speedKi = Skiri + out;
    speedKa = Skanan - out;

    if (speedKi >= 255)
      speedKi = 250;
    if (speedKa >= 255)
      speedKa = 250;
    if (speedKi <= 20)
      speedKi = 0;
    if (speedKa <= 20)
      speedKa = 0;

    // motor jalan
    if (sensor == ff)
    {
      if (errorB == 100 || errorB == -100)
        majuspeed(0, 0);
      else
        majuspeed(speedKa, speedKi);
    }

    else if (sensor == bb)
    {
      if (errorB == 100 || errorB == -100)
        mundurspeed(0, 0);
      else
        mundurspeed(speedKa, speedKi);
    }

    else
    {
      Serial.println("Else");
    }
  }
  majuspeed(0, 0);
}

void bkanan(int speed, bool sensor, int rem)
{
  float error;
  bool isFind = false;
  while (!isFind)
  {
    if (sensor == ff)
      error = errorlineF(ff);
    if (sensor == bb)
      error = errorlineF(bb) * -1;
    pkanan(speed);
    if (error == 0)
    {
      pkiriT(speed, rem);
      isFind = true;
    }
  }
  majuspeed(0, 0);
}

void bkiri(int speed, bool sensor, int rem)
{
  float error;
  bool isFind = false;
  while (!isFind)
  {
    if (sensor == ff)
      error = errorlineF(ff);
    if (sensor == bb)
      error = errorlineF(bb) * -1;
    pkiri(speed);
    if (error == 0)
    {
      pkananT(speed, rem);
      isFind = true;
    }
  }
  majuspeed(0, 0);
}
// <---- Akhir Fungsi Line Follower motor disini --->
// --------------------------------------------------
