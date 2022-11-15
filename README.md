# mazeV1
Program maze solving GR

#Library yang dibutuhkan
1. <GyverOLED.h>
2. <analogWrite.h>
3. <EEPROM.h>
4. <Wire.h>
5. <Adafruit_PWMServoDriver.h>

Example
1. Format program => Format program untuk library mazeV1
2. example_auto_misi => contoh example program untuk MRC2022 untuk track SMP
3. SMA_MRC_2022 => contoh example program untuk MRC2022 untuk track SMA

Contoh fungsi yang digunakan

Fungsi Fungsi Pada Robot Maze Solving MRC 2022
Setting Global
ff -> Sensor Depan / Maju
bb -> Sendor Belakang / Mundur
hitam -> warna garis hitam
putih -> warna garis putih
1000 ms = 1 s / 1 detik

1.	pkiriT(speed, timer)
Robot akan berputar ke kiri dengan kecepatan (speed( selama waktu (timer) yang di tentukan.
contoh => pkiriT(100 , 5000)  maka robot akan berputar ke kiri selama 5000 milisecond (5 detik) dan kecepatan 100

2.	pKanan(speed, timer)
Robot akan berputar ke kanan dengan kecepatan (speed( selama waktu (timer) yang di tentukan.
contoh => pkananT(100 , 5000)  maka robot akan berputar ke kiri selama 5000 milisecond (5 detik) dan kecepatan 100

3.	majutimer(speed_kanan, speed_kiri, timer)
Robot akan maju dengan speed (speed_kanan dan speed_kiri) kanan kiri yang telah diatur dan akan maju selama waktu yang telah diatur (timer)
Contoh => majutimer(100, 100, 5000) maka robot akan maju dengan kecepatan motor kanan = speed_kanan(100) , motor_kiri = speed_kiri(100) dan selama waktu = timer.

4.	mundurtimer(speed_kanan, speed_kiri, timer)
Robot akan mundur dengan speed (speed_kanan dan speed_kiri) kanan kiri yang telah diatur dan akan mundur selama waktu yang telah diatur (timer)
Contoh => mundurtimer(100, 100, 5000) maka robot akan mundur dengan kecepatan motor kanan = speed_kanan(100) , motor_kiri = speed_kiri(100) dan selama waktu = timer.

5.	linefollower(speed_kiri, speed_kanan, sensor_yang_digunakan, warna_garis)
Robot akan mengikuti garis secara terus menerus.
Contoh => linefollower(100, 100, ff, hitam) maka robot akan mengikuti garis dengan arah maju dan garis berwarna hitam.
6.	lfEncoder(speed_kiri, speed_kanan, sensor_yang_digunakan , jarak(mm), rem, warna_garis)
Robot akan mengikuti garis dengan jarak yang ditentukan, jadi robot akan berjalan mengikuti garis dengan jarak yang ditentukan.
Contoh => lfEncoder(100, 100, ff, 100, 20, hitam) maka robot akan mengikuti garis dan akan berhenti sesuai jarak yang ditentukan yaitu 100 mm atau 10 cm

7.	linecrossfind(speed_kiri, speed_kanan, sensor_yang_digunakan, rem, warna_garis)
Robot akan mengikuti garis sampai menemukan sebuah persimpangan (pertigaan atau perempatan) Contoh => linecrossfind(100, 100, ff, 20, hitam)

8.	linefindEight(speed_kiri, speed_kanan, sensor_yang_digunakan, rem, warna_garis)
Robot akan mengikuti garis sampai 4 sensor yang berada di tengah mendeteksi garis. Tujuanya untuk bisa berhenti pada kotak.
Contoh => linefindEight(100, 100, ff, 20, hitam)

9.	lfDelay(speed_kiri, speed_kanan, sensor_yang_digunakan, warna_garis,rem,timer(ms) )
Robot akan berjalan mengikuti garis sesuai waktu atau timer(ms) yang ditentukan. 
Contoh => lfDelay(100,100,ff, putih, 10,3000)

10.	motorEnc(speed, arah, jarakkanan(mm), jarakkiri(mm), rem)
Robot akan maju atau mundur sesuai arah dan akan berhenti saat jarak sudah tercapai
Contoh => motorEnc(100, ff,100, 100, 10) maka robot akan maju dengan sesuai jarak yang ditentukan yaitu 100 mm atau 10 cm

11.	motorEnc(speed, arah, jarakkanan(mm), jarakkiri(mm), rem)
Robot akan maju atau mundur sesuai arah dan akan berhenti saat jarak sudah tercapai
Contoh => motorEnc(100, ff,100, 100, 10) maka robot akan maju dengan sesuai jarak yang ditentukan yaitu 100 mm atau 10 cm

12.	belokEnc(speed,  arahBelok, jarakkiri, jarakkanan)
Robot akan berputan ke kiri atau ke kanan dan akan berhenti saat jarak sudah terpenuhi.
Contoh => belokEnc(100, kiri, 50, 50) maka robot akan berputar ke kiri dengan jarak 50 mm atau 5 cm dengan speed 100.

13.	findObject(speed_kiri, speed_kanan, jarak,  rem)
Robot akan maju dan berhenti saat sensor jarak mendeteksi object dengan jarak (mm) yang ditentukan. 
Contoh => findObject(100,100,500,10) maka robot akan berjalan maju dan akan berhenti apabila mendeteksi object didepanya sejauh jarak yang ditentukan yaitu 500 mm (50 cm).




