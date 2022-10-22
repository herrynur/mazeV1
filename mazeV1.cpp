#include "mazeV1.h"

// oled
GyverOLED<SSD1306_128x32, OLED_BUFFER> oled;
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

#define pid1() pidvalue(0.04, 0.00, 1.25)  // speed 100
#define pid2() pidvalue(0.045, 0.00, 1.75) // pid sik gendeng

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
  pinMode(btn1, INPUT);
  pinMode(btn2, INPUT);
  pinMode(btn3, INPUT);
  pinMode(btn4, INPUT);
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
  oled.clear();
  oled.update();
  oled.setCursor(5, 1);
  oled.setScale(1);
  oled.print("Maze V1 Initializing");
  oled.update();
  delay(2000);
  oled.clear();
  oled.update();
  // encoder
  pinMode(encKn, INPUT);
  pinMode(encKr, INPUT);
  pinMode(led, OUTPUT);
  // custom init
  mazeInit();
}

// PID
void pidvalue(float kp_, float ki_, float kd_)
{
  kp = kp_;
  kd = kd_;
  ki = ki_;
}

// <---- Fungsi Pembacaan Button disini --->
// --------------------------------------------------
bool _button(int pin)
{
  if (digitalRead(pin) == LOW)
  {
    return true;
  }
}
// <---- Akhir Fungsi Pembacaan Button disini --->
// --------------------------------------------------

// <---- Fungsi Pembacaan Sensor disini --->
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
  /////////////////////////////////////
  for (int i = 0; i <= 11; i++)
  {
    if (s[i] >= 3300)
    {
      hasil_adc[i] = 1;
    }
    else
    {
      hasil_adc[i] = 0;
    }
  }

  oled.setScale(1);
  oled.setCursorXY(0, 4);
  oled.print("F = ");
  for (int i = 0; i <= 11; i++)
  {
    oled.print(hasil_adc[i]);
    Serial.print(s[i]);
    Serial.print("-");
  }
  Serial.println();
  oled.update();

  // cek nilai analog
  //   for (int i = 0; i <= 11; i++)
  //  {
  //    Serial.print(s[i]);
  //    Serial.print("-");
  //  }
  //  Serial.println();

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

  /////////////////////////////////////
  for (int i = 0; i <= 11; i++)
  {
    if (s[i] >= 3300)
    {
      hasil_adc[i] = 1;
    }
    else
    {
      hasil_adc[i] = 0;
    }
  }

  oled.setScale(1);
  oled.setCursorXY(0, 16);
  oled.print("R = ");
  for (int i = 11; i >= 0; i--)
  {
    oled.print(hasil_adc[i]);
    // Serial.print(hasil_adc[i]);
  }
  // Serial.println();
  oled.update();
}

int errorlineB(bool kondisi)
{
  int errorline = 0;

  // if (kondisi == ff)
  //   errorline = errorF(ff);
  // if (kondisi == bb)
  //   errorline = errorF(bb);

  errorline = errorF(kondisi);

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
  // case 0b000000000000:
  //   error_ = 100;
  //   break;
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

int errorlineW(bool kondisi)
{
  int errorline = 0;

  errorline = errorF(kondisi);

  int16_t error_ = 0;
  switch (errorline)
  {
  case 0b000001111111:
    error_ = -90;
    break;
  case 0b000011111111:
    error_ = -100;
    break;
  case 0b011111111111:
    error_ = -76;
    break;
  case 0b001111111111:
    error_ = -60;
    break;
  case 0b101111111111:
    error_ = -46;
    break;
  case 0b100111111111:
    error_ = -34;
    break;
  case 0b110111111111:
    error_ = -24;
    break;
  case 0b110011111111:
    error_ = -16;
    break;
  case 0b111011111111:
    error_ = -10;
    break;
  case 0b111001111111:
    error_ = -6;
    break;
  case 0b111101111111:
    error_ = -4;
    break;
  case 0b111100111111:
    error_ = -2;
    break;
  case 0b111110011111:
    error_ = 0;
    break;
  case 0b111111011111:
    error_ = 0;
    break;
  case 0b111110111111:
    error_ = 0;
    break;
  case 0b111110001111:
    error_ = 1;
    break;
  case 0b111111001111:
    error_ = 2;
    break;
  case 0b111111101111:
    error_ = 4;
    break;
  case 0b111111100111:
    error_ = 6;
    break;
  case 0b111111110111:
    error_ = 10;
    break;
  case 0b111111110011:
    error_ = 16;
    break;
  case 0b111111111011:
    error_ = 24;
    break;
  case 0b111111111001:
    error_ = 34;
    break;
  case 0b111111111101:
    error_ = 46;
    break;
  case 0b111111111100:
    error_ = 60;
    break;
  case 0b111111111110:
    error_ = 76;
    break;
  case 0b111111110000:
    error_ = 100;
    break;
  case 0b111111100000:
    error_ = 90;
    break;
    // lostline
  // case 0b000000000000:
  //   error_ = 100;
  //   break;
  // crossfind
  case 0b000000000000:
    error_ = 200;
    break;
  case 0b111110000000:
    error_ = 50;
    break;
  case 0b000000111111:
    error_ = -50;
    break;
  }
  return error_;
}

float newErrorlineW(bool kondisi)
{
  int errorline = 0;

  errorline = errorF(kondisi);

  int16_t error_ = 0;
  switch (errorline)
  {
  case 0b001111111111:
    error_ = -62;
    break;
  case 0b000111111111:
    error_ = -54;
    break;
  case 0b100111111111:
    error_ = -45;
    break;
  case 0b100011111111:
    error_ = -36;
    break;
  case 0b110011111111:
    error_ = -28;
    break;
  case 0b110000111111:
    error_ = -20;
    break;
  case 0b110001111111:
    error_ = -12;
    break;
  case 0b111000111111:
    error_ = -8;
    break;
  case 0b111000011111:
    error_ = -6;
    break;
  case 0b111100001111:
    error_ = 0;
    break;
  case 0b111110000111:
    error_ = 6;
    break;
  case 0b111111000111:
    error_ = 8;
    break;
  case 0b111111000011:
    error_ = 12;
    break;
  case 0b111111100011:
    error_ = 20;
    break;
  case 0b111111110011:
    error_ = 28;
    break;
  case 0b111111110001:
    error_ = 36;
    break;
  case 0b111111111001:
    error_ = 45;
    break;
  case 0b111111111000:
    error_ = 54;
    break;
  case 0b111111111100:
    error_ = 62;
    break;
  case 0b111111111111:
    error_ = 100;
    break;
  }
  // Serial.println(error_);
  return error_;
}

float newErrorlineB(bool kondisi)
{
  int errorline = 0;

  errorline = errorF(kondisi);

  int16_t error_ = 0;
  switch (errorline)
  {
  case 0b0111111111:
    error_ = -46;
    break;
  case 0b0011111111:
    error_ = -34;
    break;
  case 0b0001111111:
    error_ = -24;
    break;
  case 0b0000111111:
    error_ = -16;
    break;
  case 0b1000011111:
    error_ = -10;
    break;
  case 0b110001111111:
    error_ = -6;
    break;
  case 0b110000111111:
    error_ = -4;
    break;
  case 0b111000011111:
    error_ = -2;
    break;
  case 0b111100001111:
    error_ = 0;
    break;
  case 0b111110000111:
    error_ = 2;
    break;
  case 0b111111000011:
    error_ = 4;
    break;
  case 0b111111100011:
    error_ = 6;
    break;
  case 0b111111100001:
    error_ = 10;
    break;
  case 0b111111110000:
    error_ = 16;
    break;
  case 0b111111111000:
    error_ = 24;
    break;
  case 0b111111111100:
    error_ = 34;
    break;
  case 0b111111111110:
    error_ = 46;
    break;
  }
  return error_;
}

bool detectcross(bool sensor, bool warna)
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

  if (warna == hitam)
  {
    if (s[0] > 3300 || s[1] > 3300)
    {
      hasil = true;
    }
    else
    {
      hasil = false;
    }
  }
  if (warna == putih)
  {
    if (s[0] < 2000 || s[1] < 2000)
    {
      hasil = true;
    }
    else
    {
      hasil = false;
    }
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
    if (s[i] >= 3300)
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
// <---- Akhir Fungsi Pembacaan Sensor --->
// --------------------------------------------------

// <---- Fungsi Encoder motor disini --->
// ----------------------------------------

void IRAM_ATTR pulseCountKn()
{
  counterKn++;
}

void IRAM_ATTR pulseCountKr()
{
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
  // keliling 138 mm
  // 0.6714
  // Satu putaran 210 pulsa
  float pulse = pulsaKn();
  float mm = pulse * 0.65714;
  return mm;
}

float mmKr()
{
  // keliling 138 mm
  // 0.6714
  float pulse = pulsaKr();
  float mm = pulse * 0.65714;
  return mm;
}

void motorEnc(int speed, bool arah, int jkanan, int jkiri, int brake)
{
  bool isComplete = false;
  bool Ckiri = false;
  bool Ckanan = false;
  float _kiri = 0;
  float _kanan = 0;
  while (!isComplete)
  {
    _kiri = mmKr();
    _kanan = mmKn();

    delay(10);

    if (arah == _maju)
    {
      mtrknMj(speed);
      mtrkrMj(speed);
    }
    if (arah == _mundur)
    {
      mtrknMn(speed);
      mtrkrMn(speed);
    }

    if (_kiri >= jkiri)
      Ckiri = true;
    if (_kanan >= jkanan)
      Ckanan = true;

    if (Ckiri == true || Ckanan == true)
    {
      motorBerhenti();
      if (arah == _maju)
        mundurremS(brake);
      if (arah == _mundur)
        majuremS(brake);
      isComplete = true;
    }
  }

  counterKn = 0;
  counterKr = 0;
  lastTimeKn = 0;
  lastTimeKr = 0;
}

void belokEnc(int speed, bool arahBelok, int jaraKiri, int jaraKanan)
{
  bool isComplete = false;
  bool Ckiri = false;
  bool Ckanan = false;
  float _kiri = 0;
  float _kanan = 0;
  while (!isComplete)
  {
    _kiri = mmKr();
    _kanan = mmKn();

    delay(10);

    if (arahBelok)
    {
      pkiri(speed);
    }
    if (!arahBelok)
    {
      pkanan(speed);
    }

    if (_kiri >= jaraKiri)
      Ckiri = true;
    if (_kanan >= jaraKanan)
      Ckanan = true;

    if (Ckiri == true || Ckanan == true)
    {
      motorBerhenti();
      isComplete = true;
    }
  }
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
}

void motorBerhenti()
{
  maju(0);
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
      motorBerhenti();
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
      motorBerhenti();
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
void findCross(int speed, bool sensor, bool warna, int rem)
{
  bool isFind = false;
  bool detected = false;
  while (!isFind)
  {
    detected = detectcross(sensor, warna);
    if (sensor == ff)
      majuspeed(speed, speed);
    if (sensor == bb)
      majuspeed(-speed, -speed);
    if (detected)
    {
      motorBerhenti();
      isFind = true;
    }
  }
  if (sensor == ff)
    mundurremS(rem);
  if (sensor == bb)
    majuremS(rem);
}
void linefollower(int Skiri, int Skanan, bool sensor, bool warna)
{
  float errorB, error, lasterror = 0, sumerror = 0;
  // float KP = 0.038, KI = 0.0, KD = 3.00;
  float KP = kp, KI = ki, KD = kd;
  float P, I, D, out;
  float speedKa, speedKi;

  while (true)
  {
    if (warna == hitam)
    {
      if (sensor == ff)
        error = newErrorlineB(ff);
      if (sensor == bb)
        error = newErrorlineB(bb) * -1;
    }
    if (warna == putih)
    {
      if (sensor == ff)
        error = newErrorlineW(ff);
      if (sensor == bb)
        error = newErrorlineW(bb) * -1;
    }

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
  }
}

void linecrossfind(int Skiri, int Skanan, bool sensor, int rem, bool warna)
{
  float errorBf, error, lasterror = 0, sumerror = 0;
  float KP = kp, KI = ki, KD = kd;
  float P, I, D, out;
  float speedKa, speedKi;
  bool errorB;
  bool detected = false;

  bool isFind = false;
  while (!isFind)
  {
    if (warna == hitam)
    {
      if (sensor == ff)
        error = newErrorlineB(ff);
      if (sensor == bb)
        error = newErrorlineB(bb) * -1;
    }
    if (warna == putih)
    {
      if (sensor == ff)
        error = newErrorlineW(ff);
      if (sensor == bb)
        error = newErrorlineW(bb) * -1;
    }

    error = newErrorlineW(ff);
    detected = detectcross(sensor, warna);

    // Serial.println(error);
    // oled.setScale(1);
    // oled.setCursorXY(0, 24);
    // oled.print("Error = ");
    // oled.print(error);
    // oled.update();

    errorBf = error;

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
      if (detected)
      {
        majuspeed(0, 0);
        isFind = true;
      }
      else
        majuspeed(speedKa, speedKi);
    }

    if (sensor == bb)
    {
      if (detected)
      {
        majuspeed(0, 0);
        isFind = true;
      }
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

void noLinefind(int Skiri, int Skanan, bool sensor, bool warna, int rem)
{
  float errorB, error, lasterror = 0, sumerror = 0;
  float KP = kp, KI = ki, KD = kd;
  float P, I, D, out;
  float speedKa, speedKi;

  bool isFind = false;
  while (!isFind)
  {
    if (warna == hitam)
    {
      if (sensor == ff)
        error = newErrorlineB(ff);
      if (sensor == bb)
        error = newErrorlineB(bb) * -1;
    }
    if (warna == putih)
    {
      if (sensor == ff)
        error = newErrorlineW(ff);
      if (sensor == bb)
        error = newErrorlineW(bb) * -1;
    }

    errorB = error;
    if (errorB == 100)
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
  if (rem < 0)
    mundurtimer(Skanan, Skiri, rem * -1);
  if (rem > 0)
    majutimer(Skanan, Skiri, rem);
  majuspeed(0, 0);
}

void lfEncoder(int Skiri, int Skanan, bool sensor, int jarak, int rem, bool warna)
{
  float errorBf, error, lasterror = 0, sumerror = 0;
  float KP = kp, KI = ki, KD = kd;
  float P, I, D, out;
  float speedKa, speedKi;
  bool errorB;
  float mtKr = 0;
  float mtKn = 0;

  bool isComplete = false;
  while (!isComplete)
  {
    mtKr = mmKr();
    mtKn = mmKn();

    if (warna == hitam)
    {
      if (sensor == ff)
        error = newErrorlineB(ff);
      if (sensor == bb)
        error = newErrorlineB(bb) * -1;
    }
    if (warna == putih)
    {
      if (sensor == ff)
        error = newErrorlineW(ff);
      if (sensor == bb)
        error = newErrorlineW(bb) * -1;
    }

    errorBf = error;

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

    if (mtKr >= jarak || mtKn >= jarak)
      isComplete = true;

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
      majuspeed(speedKa, speedKi);
    }

    else if (sensor == bb)
    {
      majuspeed(speedKa * -1, speedKi * -1);
    }

    else
    {
      Serial.println("Else");
    }
  }
  if (sensor == _maju)
    mundurremS(rem);
  if (sensor == _mundur)
    majuremS(rem);
}

void bkanan(int speed, bool sensor, int rem)
{
  float error;
  bool isFind = false;
  while (!isFind)
  {
    if (sensor == ff)
      error = errorlineB(ff);
    if (sensor == bb)
      error = errorlineB(bb) * -1;
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
      error = errorlineB(ff);
    if (sensor == bb)
      error = errorlineB(bb) * -1;
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
