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

int putih_[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int hitam_[14] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};
int adcrange[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// servo general
#define SERVO_FREQ 50

// servo capit
#define capitMIN 600
#define capitMAX 2400
uint8_t capit = 0;

// servo updown
#define updownMIN 10
#define updownMAX 600
uint8_t UpDown = 1;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// huskylens
int idResult = 0;
HUSKYLENS huskylens;

// maze loop
bool flag = false;
bool start = false;
bool startBtn = false;
bool calibrate = false;
bool calibrateBtn = false;

// button
bool button1 = false;
bool button2 = false;
bool button3 = false;
bool button4 = false;
bool buttoncal = false;

// Sensor jarak
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Mode
uint16_t mode = 0;
bool modeKanan = false;
bool modeKiri = false;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  EEPROM.begin(14);
  for (int i = 0; i <= 11; i++)
  {
    adcrange[i] = EEPROM.read(i) * 16;
    delay(10);
  }
  oled.init();
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

  digitalWrite(sel0, LOW);
  digitalWrite(sel1, LOW);
  digitalWrite(sel2, LOW);
  digitalWrite(sel3, LOW);
  digitalWrite(EN_F, HIGH);
  digitalWrite(EN_R, HIGH);

  // led
  pinMode(led, OUTPUT);
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

  // custom init
  mazeInit();

  // servo
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  // motor
  pinMode(naikPin, OUTPUT);
  pinMode(turunPin, OUTPUT);

  // Sensor jarak
  if (!lox.begin())
  {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }
}

// PID
void pidvalue(float kp_, float ki_, float kd_)
{
  kp = kp_;
  kd = kd_;
  ki = ki_;
}

// <----- Fungsi Huskylens Disini ----->
// ------------------------------------

int resultObject(int delay_)
{
  bool isComplete = false;
  idResult = 0;
  unsigned long timeStart = millis();

  while (!huskylens.begin(Serial2))
  {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }
  delay(100);
  huskylens.writeAlgorithm(ALGORITHM_OBJECT_CLASSIFICATION);
  while (!isComplete)
  {
    huskyRead();
    if ((unsigned long)millis() - timeStart > delay_)
    {
      isComplete = true;
    }
  }
  return idResult;
}

void huskyRead()
{
  if (!huskylens.request())
    Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  else if (!huskylens.isLearned())
    Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
  else if (!huskylens.available())
    Serial.println(F("No block or arrow appears on the screen!"));
  else
  {
    while (huskylens.available())
    {
      HUSKYLENSResult result = huskylens.read();
      printResult(result);
    }
  }
}

void printResult(HUSKYLENSResult result)
{
  if (result.command == COMMAND_RETURN_BLOCK)
  {
    // Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
    idResult = result.ID;
  }
  else if (result.command == COMMAND_RETURN_ARROW)
  {
    // Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
    idResult = result.ID;
  }
  else
  {
    Serial.println("Object unknown!");
  }
}

// <----- Akhir Fungsi Huskylens Disini ----->
// -------------------------------------------

// <---- Fungsi Servo disini --->
// --------------------------------------------------

// Servo SMP
void capitOpen()
{
  pwm.setPWM(0, 0, 250);
  delay(500);
}

void capitClose()
{
  pwm.setPWM(0, 0, 500);
  delay(500);
}

void servoPickup()
{
  pwm.setPWM(1, 0, 370);
  delay(500);
}

void servoPickDown()
{
  pwm.setPWM(1, 0, 100);
  delay(500);
}

void lempar()
{
  pwm.setPWM(2, 0, 100);
  delay(3000);
  pwm.setPWM(2, 0, 0);
}

// Servo SMA
void capit_SMA(bool kondisi)
{
  if (kondisi)
  {
    pwm.setPWM(2, 0, 200);
    delay(100);
  }
  if (!kondisi)
  {
    pwm.setPWM(2, 0, 370);
    delay(100);
  }
}

void standBy_SMA()
{
  digitalWrite(naikPin, 0);
  digitalWrite(turunPin, 0);
}

void picker_SMA(bool kondisi)
{
  if (kondisi)
  {
    pwm.setPWM(1, 0, 350);
    delay(100);
  }
  if (!kondisi)
  {
    pwm.setPWM(1, 0, 175);
    delay(100);
  }
}

void lempar_SMA(int delay_)
{
  pwm.setPWM(3, 0, 400);
  delay(delay_);
  pwm.setPWM(3, 0, 0);
}

void updown_SMA(bool kondisi, int levelDelay)
{
  if (kondisi)
  {
    digitalWrite(naikPin, 1);
    digitalWrite(turunPin, 0);
    delay(levelDelay);
    standBy_SMA();
  }
  if (!kondisi)
  {
    digitalWrite(naikPin, 0);
    digitalWrite(turunPin, 1);
    delay(levelDelay);
    standBy_SMA();
  }
}

void updown_SMA_1(bool kondisi)
{
  if (kondisi)
  {
    digitalWrite(naikPin, 1);
    digitalWrite(turunPin, 0);
    standBy_SMA();
  }
  if (!kondisi)
  {
    digitalWrite(naikPin, 0);
    digitalWrite(turunPin, 1);
    standBy_SMA();
  }
}
// <---- Akhir Fungsi Servo disini --->
// --------------------------------------------------

// <---- Fungsi Pembacaan Button disini --->
// --------------------------------------------------
bool readButton(uint16_t pin)
{
  bool read_ = false;
  int cnt = 0;

  if (digitalRead(pin) == HIGH)
  {
    while (digitalRead(pin) == HIGH)
    {
      cnt++;
      delay(10);
      if (cnt >= 90)
        return true;
    }
  }
  else
    return false;
}

bool calbutton()
{
  bool read_ = false;
  int cnt = 0;

  if (digitalRead(btn1) == HIGH)
  {
    while (digitalRead(btn1) == HIGH)
    {
      cnt++;
      delay(10);
      if (cnt >= 500)
        return true;
    }
  }
  else
    return false;
}
// <---- Akhir Fungsi Pembacaan Button disini --->
// --------------------------------------------------

// <---- Fungsi Pembacaan Sensor disini --->
// --------------------------------------------------
void kalibrasi()
{
  for (int i = 0; i <= 11; i++)
  {
    readmuxF();

    if (s[i] < putih_[i])
      putih_[i] = s[i];
    if (s[i] > hitam_[i])
      hitam_[i] = s[i];

    adcrange[i] = putih_[i] + ((hitam_[i] - putih_[i]) / 2);
    hasil_adc[i] = s[i];
    EEPROM.write(i, adcrange[i] / 16);
    EEPROM.commit();
    if (hasil_adc[i] > adcrange[i])
    {
      hasil_adc[i] = 1;
    }
    else
    {
      hasil_adc[i] = 0;
    }
  }
  Serial.print("HASIL KALIBRASI   = ");
  for (int i = 0; i <= 11; i++)
  {
    Serial.print(hasil_adc[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}
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
    if (s[i] >= adcrange[i])
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
    Serial.print(hasil_adc[i]);
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
    if (s[i] >= adcrange[i])
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
    Serial.print(hasil_adc[i]);
    Serial.print("-");
  }
  Serial.println();
  oled.update();
}

int errorKhusus(bool kondisi)
{
  int errorline = 0;
  errorline = errorF(kondisi);

  int16_t error_ = 0;
  switch (errorline)
  {
  case 0b111111000000:
    error_ = 100;
    break;
  case 0b011111100000:
    error_ = 100;
    break;
  case 0b001111110000:
    error_ = 100;
    break;
  case 0b000111111000:
    error_ = 100;
    break;
  case 0b000011111100:
    error_ = 100;
    break;
  case 0b000001111110:
    error_ = 100;
    break;
  case 0b000000111111:
    error_ = 100;
    break;

  case 0b111110000000:
    error_ = 100;
    break;
  case 0b011111000000:
    error_ = 100;
    break;
  case 0b001111100000:
    error_ = 100;
    break;
  case 0b000111110000:
    error_ = 100;
    break;
  case 0b000011111000:
    error_ = 100;
    break;
  case 0b000001111100:
    error_ = 100;
    break;
  case 0b000000111110:
    error_ = 100;
    break;
  case 0b000000011111:
    error_ = 100;
    break;
  }
  return error_;
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
  case 0b000000000111:
    error_ = 22;
    break;
  case 0b000000001110:
    error_ = 14;
    break;
  case 0b000000011100:
    error_ = 8;
    break;
  case 0b000000111000:
    error_ = 2;
    break;
  case 0b000001110000:
    error_ = 0;
    break;
  case 0b000011100000:
    error_ = 0;
    break;
  case 0b000111000000:
    error_ = -2;
    break;
  case 0b001110000000:
    error_ = -8;
    break;
  case 0b011100000000:
    error_ = -14;
    break;
  case 0b111000000000:
    error_ = -22;
    break;

  case 0b000000000001:
    error_ = 22;
    break;
  case 0b000000000011:
    error_ = 18;
    break;
  case 0b000000000010:
    error_ = 16;
    break;
  case 0b000000000110:
    error_ = 14;
    break;
  case 0b000000000100:
    error_ = 12;
    break;
  case 0b000000001100:
    error_ = 10;
    break;
  case 0b000000001000:
    error_ = 8;
    break;
  case 0b000000011000:
    error_ = 6;
    break;
  case 0b000000010000:
    error_ = 4;
    break;
  case 0b000000110000:
    error_ = 2;
    break;

  case 0b000000100000:
    error_ = 1;
    break;
  case 0b000001100000:
    error_ = 0;
    break;
  case 0b000001000000:
    error_ = -1;
    break;

  case 0b000011000000:
    error_ = -2;
    break;
  case 0b000010000000:
    error_ = -4;
    break;
  case 0b000110000000:
    error_ = -6;
    break;
  case 0b000100000000:
    error_ = -8;
    break;
  case 0b001100000000:
    error_ = -10;
    break;
  case 0b001000000000:
    error_ = -12;
    break;
  case 0b011000000000:
    error_ = -14;
    break;
  case 0b010000000000:
    error_ = -16;
    break;
  case 0b110000000000:
    error_ = -18;
    break;
  case 0b100000000000:
    error_ = -22;
    break;
  case 0b000000000000:
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
  case 0b011111111111:
    error_ = -16;
    break;
  case 0b001111111111:
    error_ = -14;
    break;
  case 0b000111111111:
    error_ = -12;
    break;
  case 0b000011111111:
    error_ = -10;
    break;
  case 0b100001111111:
    error_ = -8;
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
    error_ = 8;
    break;
  case 0b111111110000:
    error_ = 10;
    break;
  case 0b111111111000:
    error_ = 12;
    break;
  case 0b111111111100:
    error_ = 14;
    break;
  case 0b111111111110:
    error_ = 16;
    break;
  }
  return error_;
}

bool detectcross(bool sensor, bool warna)
{
  int s[3];
  bool hasil;
  bool detectL = false;
  bool detectR = false;
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
    if (s[0] >= adcrange[0] || s[1] >= adcrange[1])
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
    if (s[0] <= adcrange[0] || s[1] <= adcrange[1])
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

bool detectCenter(bool sensor, bool warna)
{
  int s[4];
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

  digitalWrite(sel0, 0);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 0);
  s[0] = analogRead(ADC);
  // 1000
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 0);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 0);
  s[1] = analogRead(ADC);
  // 0100
  digitalWrite(sel0, 0);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 0);
  digitalWrite(sel3, 0);
  s[2] = analogRead(ADC);
  // 1110
  digitalWrite(sel0, 1);
  digitalWrite(sel1, 1);
  digitalWrite(sel2, 1);
  digitalWrite(sel3, 0);
  s[3] = analogRead(ADC);

  if (warna == hitam)
  {
    if (s[0] > adcrange[0] && s[1] > adcrange[1] && s[2] > adcrange[2] && s[3] > adcrange[3])
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
    if (s[0] < adcrange[0] && s[1] < adcrange[1] && s[2] < adcrange[2] && s[3] < adcrange[3])
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
    if (s[i] >= adcrange[i])
    {
      hasil_adc[i] = 0;
    }
    else
    {
      hasil_adc[i] = 1;
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

    if (_kiri >= jkiri)
      Ckiri = true;
    if (_kanan >= jkanan)
      Ckanan = true;

    if (Ckiri == true || Ckanan == true)
    {
      motorBerhenti();
      if (arah == ff)
        mundurremS(brake);
      if (arah == bb)
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
      motorBerhenti();
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
      motorBerhenti();
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
      motorBerhenti();
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
    majuzero(-Skanan, -Skiri);
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
    I = 0;
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

    if (speedKa >= Skanan)
      speedKa = Skanan;
    if (speedKi >= Skiri)
      speedKi = Skiri;

    // motor jalan
    if (sensor == ff)
    {
      majuspeed(speedKa, speedKi);
    }

    else if (sensor == bb)
    {
      mundurspeed(speedKa, speedKi);
    }

    else
    {
      Serial.println("Else");
    }
  }
}

void linefindEight(int Skiri, int Skanan, bool sensor, int rem, bool warna)
{
  float errorBf, error, lasterror = 0, sumerror = 0;
  float KP = kp, KI = ki, KD = kd;
  float P, I, D, out;
  float speedKa, speedKi;
  bool errorB;
  int detected = 0;

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

    if (speedKa >= Skanan)
      speedKa = Skanan;
    if (speedKi >= Skiri)
      speedKi = Skiri;

    // motor jalan
    if (sensor == ff)
    {
      detected = errorKhusus(ff);
      if (detected == 100)
      {
        motorBerhenti();
        mundurtimer(Skanan, Skiri, rem);
        isFind = true;
      }
      else
        majuspeed(speedKa, speedKi);
    }

    if (sensor == bb)
    {
      detected = errorKhusus(bb);
      if (detected == 100)
      {
        motorBerhenti();
        majutimer(Skanan, Skiri, rem);
        isFind = true;
      }
      else
        majuspeed(speedKa * -1, speedKi * -1);
    }
  }
  if (sensor == ff)
    mundurtimer(Skanan, Skiri, rem);
  if (sensor == bb)
    majutimer(Skanan, Skiri, rem);
}

void lf_detectobject(int Skiri, int Skanan, bool sensor, bool warna, int rem, int _jarak)
{
  float errorBf, error, lasterror = 0, sumerror = 0;
  float KP = kp, KI = ki, KD = kd;
  float P, I, D, out;
  float speedKa, speedKi;
  bool errorB;
  int hasil = 0;

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

    if (speedKa >= Skanan)
      speedKa = Skanan;
    if (speedKi >= Skiri)
      speedKi = Skiri;

    // motor jalan
    if (sensor == ff)
    {
      hasil = jarak();
      if (hasil <= _jarak && hasil != 0)
      {
        motorBerhenti();
        isFind = true;
      }
      else
        majuspeed(speedKa, speedKi);
    }

    if (sensor == bb)
    {
      hasil = jarak();
      if (hasil <= _jarak && hasil != 0)
      {
        motorBerhenti();
        isFind = true;
      }
      else
        majuspeed(speedKa * -1, speedKi * -1);
    }
  }
  if (sensor == bb)
    mundurtimer(Skanan, Skiri, rem);
  if (sensor == ff)
    majutimer(Skanan, Skiri, rem);
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

    if (speedKa >= Skanan)
      speedKa = Skanan;
    if (speedKi >= Skiri)
      speedKi = Skiri;

    // motor jalan
    if (sensor == ff)
    {
      detected = detectcross(ff, putih);
      if (detected)
      {
        motorBerhenti();
        isFind = true;
      }
      else
        majuspeed(speedKa, speedKi);
    }

    if (sensor == bb)
    {
      detected = detectcross(bb, putih);
      if (detected)
      {
        motorBerhenti();
        isFind = true;
      }
      else
        majuspeed(speedKa * -1, speedKi * -1);
    }
  }
  if (sensor == bb)
    mundurtimer(Skanan, Skiri, rem);
  if (sensor == ff)
    majutimer(Skanan, Skiri, rem);
}

void lfDelay(int Skiri, int Skanan, bool sensor, bool warna, int rem, int delay_)
{
  float errorBf, error, lasterror = 0, sumerror = 0;
  float KP = kp, KI = ki, KD = kd;
  float P, I, D, out;
  float speedKa, speedKi;
  bool errorB;
  unsigned long timeStart = millis();

  bool isCompleted = false;
  while (!isCompleted)
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

    if (speedKa >= Skanan)
      speedKa = Skanan;
    if (speedKi >= Skiri)
      speedKi = Skiri;

    // motor jalan
    if (sensor == ff)
    {
      if ((unsigned long)millis() - timeStart > delay_)
      {
        motorBerhenti();
        isCompleted = true;
      }
      else
        majuspeed(speedKa, speedKi);
    }

    if (sensor == bb)
    {
      if ((unsigned long)millis() - timeStart > delay_)
      {
        motorBerhenti();
        isCompleted = true;
      }
      else
        majuspeed(speedKa * -1, speedKi * -1);
    }
  }
  if (sensor == bb)
    mundurtimer(Skanan, Skiri, rem);
  if (sensor == ff)
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

    if (speedKa >= Skanan)
      speedKa = Skanan;
    if (speedKi >= Skiri)
      speedKi = Skiri;

    // motor jalan
    if (sensor == ff)
    {
      if (errorB == 100 || errorB == -100)
      {
        motorBerhenti();
        isFind = true;
      }

      else
        majuspeed(speedKa, speedKi);
    }

    if (sensor == bb)
    {
      if (errorB == 100 || errorB == -100)
      {
        motorBerhenti();
        isFind = true;
      }
      else
        mundurspeed(speedKa, speedKi);
    }
  }
  if (sensor == ff)
    mundurtimer(Skanan, Skiri, rem);
  if (sensor == bb)
    majutimer(Skanan, Skiri, rem);
  motorBerhenti();
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

    if (mtKr >= jarak)
      isComplete = true;

    if (speedKi >= 255)
      speedKi = 255;
    if (speedKa >= 255)
      speedKa = 255;
    if (speedKi <= -255)
      speedKi = -255;
    if (speedKa <= -255)
      speedKa = -255;

    if (speedKa >= Skanan)
      speedKa = Skanan;
    if (speedKi >= Skiri)
      speedKi = Skiri;

    // motor jalan
    if (sensor == ff)
    {
      majuspeed(speedKa, speedKi);
    }

    else if (sensor == bb)
    {
      majuspeed(speedKa * -1, speedKi * -1);
    }
  }
  if (sensor == ff)
    mundurremS(rem);
  if (sensor == bb)
    majuremS(rem);

  counterKn = 0;
  counterKr = 0;
  lastTimeKn = 0;
  lastTimeKr = 0;
  detachInterrupt(encKr);
  detachInterrupt(encKn);
}

void bkanan(int speed, bool sensor, int rem, bool warna)
{
  bool error;
  bool isFind = false;
  pkananT(speed, 200);
  while (!isFind)
  {
    if (sensor == ff)
      error = detectCenter(ff, warna);
    if (sensor == bb)
      error = detectCenter(bb, warna);

    pkanan(speed);
    if (error)
    {
      pkiriT(speed, rem);
      isFind = true;
    }
  }
  motorBerhenti();
}

void bkiri(int speed, bool sensor, int rem, bool warna)
{
  bool error;
  bool isFind = false;
  pkiriT(speed, 200);
  while (!isFind)
  {
    if (sensor == ff)
      error = detectCenter(ff, warna);
    if (sensor == bb)
      error = detectCenter(bb, warna);

    pkiri(speed);
    if (error)
    {
      pkananT(speed, rem);
      isFind = true;
    }
  }
  motorBerhenti();
}
// <---- Akhir Fungsi Line Follower motor disini --->
// --------------------------------------------------

// <---- Fungsi utama --->
// --------------------------------------------------

void ledblink(uint16_t cnt, int delay_)
{
  for (int i = 0; i < cnt; i++)
  {
    digitalWrite(led, HIGH);
    delay(delay_);
    digitalWrite(led, LOW);
    delay(delay_);
  }
}

// <---- Fungsi jarak --->
int jarak()
{
  VL53L0X_RangingMeasurementData_t measure;
  int mResult = 0;

  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4)
  {
    mResult = measure.RangeMilliMeter;
  }
  else
  {
    Serial.println(" out of range ");
  }
  return mResult;
}

void findObject(int Skiri, int Skanan, int _jarak, int rem)
{
  int hasil = 0;
  bool isFind = false;
  while (!isFind)
  {
    hasil = jarak();
    if (hasil <= _jarak && hasil != 0)
    {
      motorBerhenti();
      isFind = true;
    }
    else
    {
      majuspeed(Skanan, Skiri);
    }
  }
  motorBerhenti();
}

// <---- Fungsi Manual Mode ---->
void manualMode(const char *addr, int speedMajuKiri, int speedMajuKanan, int speedMundurKiri, int speedMundurKanan, int speedBelok)
{
  Ps3.begin(addr);
  delay(10);
  Serial.println("Ready.");

  picker_SMA(naik);
  capit_SMA(buka);
  standBy_SMA();

  while (Ps3.isConnected())
  {
    if (Ps3.data.button.cross)
    {
      picker_SMA(turun);
    }
    else if (Ps3.data.button.square)
    {
      capit_SMA(buka);
    }
    else if (Ps3.data.button.triangle)
    {
      picker_SMA(turun);
      lempar_SMA(1000);
    }
    else if (Ps3.data.button.circle)
    {
      capit_SMA(tutup);
    }
    else if (Ps3.data.button.l1)
    {
      majuspeed(speedMajuKanan / 3, speedMajuKiri / 3);
    }
    else if (Ps3.data.button.l2)
    {
      majuspeed((speedMundurKanan * -1) / 3, (speedMundurKiri * -1 / 3));
    }
    else if (Ps3.data.button.r1)
    {
      if (Ps3.data.button.left)
        majuspeed(speedMajuKanan, speedMajuKiri / 2);
      else if (Ps3.data.button.right)
        majuspeed(speedMajuKanan / 2, speedMajuKiri);
      else
        majuspeed(speedMajuKanan, speedMajuKiri);
    }
    else if (Ps3.data.button.r2)
    {
      if (Ps3.data.button.left)
        majuspeed(speedMundurKanan * -1, (speedMundurKanan * -1) / 2);
      else if (Ps3.data.button.right)
        majuspeed((speedMundurKanan * -1) / 2, speedMundurKanan * -1);
      else
        majuspeed(speedMundurKanan * -1, speedMundurKiri * -1);
    }

    // Program maju mundur belok
    else if (Ps3.data.button.up)
    {
      majuspeed(speedMajuKanan, speedMajuKiri);
    }
    else if (Ps3.data.button.down)
    {
      majuspeed(speedMundurKanan * -1, speedMundurKiri * -1);
    }
    else if (Ps3.data.button.right)
    {
      majuspeed(speedMajuKanan / 2, speedMajuKiri);
    }
    else if (Ps3.data.button.left)
    {
      majuspeed(speedMundurKanan * -1, (speedMundurKanan * -1) / 2);
    }
    else
    {
      picker_SMA(naik);
      standBy_SMA();
      motorBerhenti();
    }
  }
}
// <---- Fungsi utama --->
void loop()
{
  motorBerhenti();
  button1 = readButton(btn1);
  button4 = readButton(btn4);

  if (button1)
  {
    capit_SMA(buka);
    modeKanan = true;
  }

  else if (button4)
  {
    capit_SMA(tutup);
    modeKiri = true;
  }

  while (modeKanan)
  {
    button1 = readButton(btn1);
    button2 = readButton(btn2);
    button3 = readButton(btn3);
    button4 = readButton(btn4);

    if (button1 && !button2 && !button3 && !button4)
      mode = 1;
    else if (button2 && !button1 && !button3 && !button4)
      mode = 2;
    else if (button3 && !button1 && !button2 && !button4)
      mode = 3;
    else if (button4 && !button1 && !button2 && !button3)
      mode = 4;
    else if (button1 && button2)
      mode = 5;
    else if (button3 && button4)
      mode = 6;
    else if (button1 && button4)
      mode = 7;

    switch (mode)
    {
    case 1:
      mode1();
      mode = 0;
      break;
    case 2:
      mode2();
      mode = 0;
      break;
    case 3:
      mode3();
      mode = 0;
      break;
    case 4:
      mode4();
      mode = 0;
      break;
    case 5:
      mode5();
      mode = 0;
      break;
    case 6:
      mode6();
      mode = 0;
      break;
    case 7:
      kalibrasi();
      digitalWrite(led, HIGH);
      button1 = readButton(btn1);
      button4 = readButton(btn4);
      if (button1 && button4)
      {
        digitalWrite(led, LOW);
        mode = 0;
      }
      break;
    }
  }

  while (modeKiri)
  {
    button1 = readButton(btn1);
    button2 = readButton(btn2);
    button3 = readButton(btn3);
    button4 = readButton(btn4);

    if (button1 && !button2 && !button3 && !button4)
      mode = 1;
    else if (button2 && !button1 && !button3 && !button4)
      mode = 2;
    else if (button3 && !button1 && !button2 && !button4)
      mode = 3;
    else if (button4 && !button1 && !button2 && !button3)
      mode = 4;
    else if (button1 && button2)
      mode = 5;
    else if (button3 && button4)
      mode = 6;
    else if (button1 && button4)
      mode = 7;

    switch (mode)
    {
    case 1:
      mode7();
      mode = 0;
      break;
    case 2:
      mode8();
      mode = 0;
      break;
    case 3:
      mode9();
      mode = 0;
      break;
    case 4:
      mode10();
      mode = 0;
      break;
    case 5:
      mode11();
      mode = 0;
      break;
    case 6:
      mode12();
      mode = 0;
      break;
    case 7:
      kalibrasi();
      digitalWrite(led, HIGH);
      button1 = readButton(btn1);
      button4 = readButton(btn4);
      if (button1 && button4)
      {
        digitalWrite(led, LOW);
        mode = 0;
      }
      break;
    }
  }
}

// OLD LOOP
// void loop()
// {
//   motorBerhenti();
//   while (!start)
//   {
//     motorBerhenti();
//     digitalWrite(led, HIGH);
//     startBtn = readButton(btn4);
//     calibrateBtn = readButton(btn1);

//     if (startBtn)
//     {
//       digitalWrite(led, LOW);
//       start = true;
//     }
//     else if (calibrateBtn)
//     {
//       calibrate = true;
//       ledblink(3);
//       while (calibrate)
//       {
//         kalibrasi();
//         calibrateBtn = readButton(btn1);
//         if (calibrateBtn)
//         {
//           ledblink(5);
//           calibrate = false;
//         }
//       }
//     }
//   }
//   mazeLoop();
//   start = false;
// }

// <---- Akhir Fungsi utama --->
// --------------------------------------------------
