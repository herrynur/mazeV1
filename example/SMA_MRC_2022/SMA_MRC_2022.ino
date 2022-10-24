#include "mazeV1.h"
//Setting PID disini
#define pid3() pidvalue(9, 0.00, 21)
//ketinggian naik per level
#define tinggi 400

bool flag = false;

void mazeInit() {
  //Panggil PID disini
  pid3();
  standBy_SMA();
}

void loop() {
  //kalibrasi();

  if (!flag)
  {

    capit_SMA(buka);
    picker_SMA(naik);
    updown_SMA(naik, tinggi);
    delay(400);

    //Menuju ke tulisan R
    linecrossfind(100, 100, ff, 50, putih);
    bkanan(50, ff, 10, putih);
    linecrossfind(100, 100, ff, 50, putih);
    linecrossfind(100, 100, ff, 50, putih);
    bkiri(50, ff, 10, putih);
    lfEncoder(70, 70, ff, 250, 10, putih);
    updown_SMA(turun, tinggi);
    delay(400);
    linefindEight(40, 40, ff, 20, putih);
    //Servo action
    capit_SMA(tutup);
    delay(400);
    updown_SMA(naik, tinggi);
    delay(400);

    //Kembali dari tulisan R menuju tulisan M
    linecrossfind(100, 100, bb, 50, putih);
    bkiri(50, ff, 10, putih);
    linecrossfind(70, 70, ff, 50, putih);
    bkanan(50, ff, 10, putih);
    lfEncoder(70, 70, ff, 250, 10, putih);
    linefindEight(40, 40, ff, 20, putih);
    capit_SMA(buka);
    delay(400);

    //Kembali dari tulisan M menuju ke C
    linecrossfind(100, 100, bb, 50, putih);
    bkanan(50, ff, 10, putih);
    linecrossfind(100, 100, ff, 50, putih);
    linecrossfind(100, 100, ff, 50, putih);
    bkiri(50, ff, 10, putih);
    lfEncoder(70, 70, ff, 250, 10, putih);
    capit_SMA(buka);
    delay(400);
    updown_SMA(turun, tinggi);
    delay(400);
    linefindEight(40, 40, ff, 20, putih);
    capit_SMA(tutup);
    delay(400);
    updown_SMA(naik, tinggi);
    delay(400);
    updown_SMA(naik, tinggi);
    delay(400);

    //Dari tulisan C menuju ke M
    linecrossfind(100, 100, bb, 50, putih);
    bkiri(50, ff, 10, putih);
    linecrossfind(100, 100, ff, 50, putih);
    linecrossfind(100, 100, ff, 50, putih);
    bkanan(50, ff, 10, putih);
    lfEncoder(70, 70, ff, 250, 10, putih);
    linefindEight(40, 40, ff, 20, putih);
    capit_SMA(buka);
    delay(400);

    //Menuju Melempar bola
    linecrossfind(100, 100, bb, 50, putih);
    bkiri(50, ff, 10, putih);
    linecrossfind(100, 100, ff, 50, putih);
    bkiri(50, ff, 10, putih);
    lfDelay(40, 40, ff, putih, 0, 300);
    lfDelay(40, 40, bb, putih, 0, 400);
    delay(400);

    //Melempar bola
    picker_SMA(turun);
    delay(500);
    picker_SMA(naik);
    delay(500);
    picker_SMA(turun);
    delay(500);
    lempar_SMA(1000);
    delay(1000);




    flag = 1;
  }
  else
    motorBerhenti();
}

//  capit_SMA(buka);
//  delay(500);
//  updown_SMA(naik, 400);
//  delay(500);
//  updown_SMA(naik, 400);
//  delay(500);
//  capit_SMA(tutup);
//  delay(500);
//  updown_SMA(turun, 400);
//  delay(500);
//  updown_SMA(turun, 400);
//  delay(500);

//  picker_SMA(naik);
//  delay(500);
//  picker_SMA(turun);
//  delay(500);
//  picker_SMA(naik);
//  delay(500);
//  picker_SMA(turun);
//  delay(500);
//  lempar_SMA(1000);
//  delay(1000);
