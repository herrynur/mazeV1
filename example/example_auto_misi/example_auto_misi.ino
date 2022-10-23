#include "mazeV1.h"
//Setting PID disini
#define pid3() pidvalue(9, 0.00, 21)

//flag
bool flag = false;

void mazeInit() {
  //Panggil PID disini
  pid3();
}

void loop() {
  if (!flag)
  {
    // Antar Botol
    antarBotol();
    // Ambil bola 1
    ambilBola(1, 260);
    // Ambil bola 2
    ambilBola(2, 600);

    flag = true;
  }
  else
    motorBerhenti();
}

void antarBotol()
{
  servoPickup();
  delay(200);
  motorEnc(50, ff, 15, 15, 10);
  lfEncoder(70, 70, ff, 585, 20, putih);
  //nyapit
  capitOpen();
  delay(200);
  servoPickDown();
  delay(200);
  capitClose();
  delay(200);

  noLinefind(90, 90, ff, putih, 100);
  capitOpen();
  delay(200);
  servoPickup();
  delay(200);
  capitClose();
  delay(200);
}

void ambilBola(int bolaKe, int delay_)
{
  //Logika bola ke 1
  if (bolaKe == 1)
  {
    linecrossfind(100, 100, bb, 50, putih);
    bkanan(50, ff, 10, putih);
  }
  //Logika bola ke 2
  if (bolaKe == 2)
  {
    linecrossfind(100, 100, ff, 50, putih);
    bkiri(50, ff, 10, putih);
  }

  linecrossfind(100, 100, ff, 80, putih);
  bkiri(50, ff, 50, putih);
  
  if (bolaKe == 1)
    lfDelay(30, 30, ff, putih, 0, delay_);
  if (bolaKe == 2)
    majutimer(40, 40, 500);
    
  capitOpen();
  delay(200);
  servoPickDown();
  delay(200);
  capitClose();
  delay(200);
  servoPickup();
  delay(200);

  if(bolaKe == 1)
  lfDelay(30, 30, bb, putih, 0, 300);
  if(bolaKe == 2)
  lfDelay(30, 30, bb, putih, 0, 650);
  
  bkiri(50, ff, 50, putih);
  linecrossfind(100, 100, ff, 100, putih);
  bkiri(50, ff, 50, putih);
  noLinefind(100, 100, bb, putih, 80);

  majutimer(40, -40, 280);
  capitOpen();
  delay(200);
  lempar();
  delay(200);
  majutimer(-40, 40, 400);
}
