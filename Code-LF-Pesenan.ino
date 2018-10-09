//### Penambahan library pada arduino IDE
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_SR.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
//=======================================//
//### Register eeprom
#define kpEeprom 0
#define kdEeprom 1
#define kiEeprom 2
#define tsEeprom 3
#define rpmEeprom 4
#define minSensEeprom 5
#define maxSensEeprom 30
#define averageEeprom 55

#define intersecEeprom 100//tapi yg asli 101 dikarenkan ditambah intersectionKe=1
#define cekPoinEeprom 400
//=======================================//
//### PenDefinisian port yang digunakan
LiquidCrystal_SR lcd(13,11,12);//port untuk LCD

#define selA A4 //untuk selectore A pada ic multiplexer
#define selB A3 //untuk selector B pada ic multiplexer
#define selC A2 //untuk selector C pada ic multiplexer
#define sensLeft  0 //PIN ADC ke 0 untuk sensor sebelah kiri (8 buah dari kiri ke kanan)
#define sensRight 1 //PIN ADC ke 1 untuk sensor sebelah kanan (4 buah setelah sensor kiri)
#define batteryMonitor 5 //PIN ADC ke 5 yang digunakan untuk membaca tegangan battery
#define tombol1   2 //tombol 4
#define tombol2   4 //tombol 3
#define tombol3   0 //tombol 2
#define tombol4   1 //tombol 1
#define pwmKanan  10 //kecepatan motor kanan
#define pwmKiri   9 //kecepatan motor kiri
#define dirLA     8 //direction A motor kiri
#define dirLB     7 //direction B motor kiri
#define dirRA     5 //direction A motor kanan
#define dirRB     6 //direction B motor kanan
#define buzzer    3 //buzzer
//=======================================//
//### variable yang digunakan
int minSementara[13],maxSementara[13],minSensor[13],maxSensor[13],eepromMirror;
int ulang,eAverage[13];
int valueLine[13],tombol[4];
char lcdBuff[16];
unsigned char Kp = 70; 
unsigned char Kd = 9;
unsigned char Ki = 3;
unsigned char Ts = 1;
unsigned char max_pwm_ref=200;
int error, last_error, MV,pid_l,pid_r,D,D1,D2,D3,I,I1,I2,I3,P,Pd;
int binerLine[12],hexaSensor,karakterBattery,mirrorIntersection;
int Max_MV;
float readADC, vout,valueBattery;
int intersectionKe=1;
int turnKe;
unsigned char nilaiPerempatan;
unsigned char valueCekPoin=1;
int awalStartBacaEeprom;
unsigned char startCPke=0;
int tandaT;
int tandaL,countBlackWhite;
unsigned char gantiBackground = 0;

//=======================================//
//### PenDefinisian emoticon pada LCD16x2
byte batteryLCDZero[8] = {  //emoticon battery habis
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111
};
byte batteryLCDOne[8] = {   //emoticon battery tinggal 1 (10-20%)
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b11111
};
byte batteryLCDTwo[8] = {   //emotican batteri tinggal 2
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
  0b11111
};
byte batteryLCDThree[8] = {   //emotican battery tinggal 3
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};
byte batteryLCDFour[8] = {    //emoticon battery tinggal 4
  0b01110,
  0b11011,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};
byte batteryLCDFive[8] = {    //emoticon battery penuh
  0b01110,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};
byte cZero[8]=                //emoticon yang digunakan untuk pembacaan sensor ketika membaca 0
{
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111, 
  B11111, 
};
byte cOne[8]=                 //emoticon yang digunakan untuk pembacaan sensor ketika membaca 1
{
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};
void setup() {
  lcd.begin(16,2);                    //pengaturan awal LCD 16x2
//### pengaturan register untuk emoticon karakter yang akan ditampilkan ke LCD
  lcd.createChar (0, cZero);        
  lcd.createChar (1, cOne);
  lcd.createChar (2, batteryLCDZero);
  lcd.createChar (3, batteryLCDOne);
  lcd.createChar (4, batteryLCDTwo);
  lcd.createChar (5, batteryLCDThree);
  lcd.createChar (6, batteryLCDFour);
  lcd.createChar (7, batteryLCDFive);
  
  lcd.home();
//=======================================//
//### pengaturan awal port yang digunakan sebagai OUTPUT atau INPUT
  for(int out=5; out<=10; out++)
  {
    pinMode(out, OUTPUT);  
  }
  pinMode(tombol1, INPUT_PULLUP);
  pinMode(tombol2, INPUT_PULLUP);
  pinMode(tombol3, INPUT_PULLUP);
  pinMode(tombol4, INPUT_PULLUP);
  pinMode(selA, OUTPUT);
  pinMode(selB, OUTPUT);
  pinMode(selC, OUTPUT);
//=======================================//
//### Nada awal ketika robot pertama kali dinyalakan atau pada saat reset
  //tone(100, 200);
  //tone(190, 200);
  //tone(255, 200);
  //tone(50, 200); 
//=======================================//
}


void loop(){
//### menu awal tampil pada LCD
 awal:
    lcd.setCursor(0,0);
    lcd.print(" 4    3   2   1 ");
    lcd.setCursor(0,1);
    lcd.print("Sen Set Path RUN");
    if(digitalRead(tombol1)==0)
    {
       delay(150); tone(255,150);lcd.clear();goto setKp;
    }       
    if(digitalRead(tombol2)==0)
    {
        delay(150); tone(255,150);lcd.clear();goto setSens;
    }
    if(digitalRead(tombol3)==0)
    {
        delay(150); tone(255,150);lcd.clear();goto runMenu; 
    }   
    if(digitalRead(tombol4) == 0)
    {
      delay(150);tone(255,150);lcd.clear(); goto setPathOrCp;                                  
    }
    goto awal;
//=======================================//
//### menu setting konstanta proportional
setKp:
  Kp=EEPROM.read(kpEeprom);
  if(digitalRead(tombol2)==0){Kp++;delay(10);lcd.clear();}
  if(digitalRead(tombol3)==0){Kp--;delay(10);lcd.clear();}
  lcd.setCursor(0,0);
  lcd.print("Bismillah...");
  lcd.setCursor(0,1);
  sprintf(lcdBuff,"Nilai KP:%i",Kp);
  lcd.print(lcdBuff);
  EEPROM.write(kpEeprom, Kp);
  delay(100);
  if(digitalRead(tombol1)==0)
  {
      delay(150);tone(255,150);lcd.clear();goto setKd;
  }
  if(digitalRead(tombol4)==0)
  {
      delay(150);tone(255,150);lcd.clear();goto awal;
  }
  goto setKp;
//=======================================//
//### menu setting konstanta derivative
setKd:
  Kd=EEPROM.read(kdEeprom);
  if(digitalRead(tombol2)==0){Kd++;delay(10);lcd.clear();}
  if(digitalRead(tombol3)==0){Kd--;delay(10);lcd.clear();}
  lcd.setCursor(0,0);
  lcd.print("Bismillah...");
  lcd.setCursor(0,1);
  sprintf(lcdBuff,"Nilai KD:%i",Kd);
  lcd.print(lcdBuff);
  EEPROM.write(kdEeprom, Kd);
  delay(100);
  if(digitalRead(tombol1)==0)
  {
      delay(150);tone(255,150);lcd.clear();goto setKi;
  }
  if(digitalRead(tombol4)==0)
  {
      delay(150);tone(255,150);lcd.clear();goto setKp;
  }
  goto setKd;
//=======================================//
//### menu setting konstanta Integral
setKi:
  Ki=EEPROM.read(kiEeprom);
  if(digitalRead(tombol2)==0){Ki++;delay(10);lcd.clear();}
  if(digitalRead(tombol3)==0){Ki--;delay(10);lcd.clear();}
  lcd.setCursor(0,0);
  lcd.print("Bismillah...");
  lcd.setCursor(0,1);
  sprintf(lcdBuff,"Nilai KI:%i",Ki);
  lcd.print(lcdBuff);
  EEPROM.write(kiEeprom, Ki);
  delay(100);
  if(digitalRead(tombol1)==0)
  {
      delay(150);tone(255,150);lcd.clear();goto setTs;
  }
  if(digitalRead(tombol4)==0)
  {
      delay(150);tone(255,150);lcd.clear();goto setKd;
  }
  goto setKi;
//=======================================//
//### menu setting konstanta Time Sampling
setTs:
  Ts=EEPROM.read(tsEeprom);
  if(digitalRead(tombol2)==0){Ts++;delay(10);lcd.clear();}
  if(digitalRead(tombol3)==0){Ts--;delay(10);lcd.clear();}
  lcd.setCursor(0,0);
  lcd.print("Bismillah...");
  lcd.setCursor(0,1);
  sprintf(lcdBuff,"Nilai TS:%i",Ts);
  lcd.print(lcdBuff);
  EEPROM.write(tsEeprom, Ts);
  delay(100);
  if(digitalRead(tombol1)==0)
  {
      delay(150);tone(255,150);lcd.clear();goto pwmReff;
  }
  if(digitalRead(tombol4)==0)
  {
      delay(150);tone(255,150);lcd.clear();goto setKi;
  }
  goto setTs;
//=======================================//
//### menu setting pwm referensi yang digunakan pada PID
pwmReff:
  max_pwm_ref=EEPROM.read(rpmEeprom);
  if(digitalRead(tombol2)==0){max_pwm_ref++;delay(10);lcd.clear();}
  if(digitalRead(tombol3)==0){max_pwm_ref--;delay(10);lcd.clear();}
  lcd.setCursor(0,0);
  lcd.print("Bismillah...");
  lcd.setCursor(0,1);
  sprintf(lcdBuff,"Pwm Reff:%3d",max_pwm_ref);
  lcd.print(lcdBuff);
  EEPROM.write(rpmEeprom, max_pwm_ref);
  delay(100);
  if(digitalRead(tombol1)==0)
  {
      delay(150);tone(255,150);lcd.clear();goto battery;
  }
  if(digitalRead(tombol4)==0)
  {
      delay(150);tone(255,150);lcd.clear();goto setTs;
  }
  goto pwmReff;
//=======================================//
//### menu yang digunakan untuk melihat sensor sebelah kanan, kiri dan untuk merecord nilai dari sensor garis
setSens:
  lcd.setCursor(2,0);
  lcd.print(" 4  3  2  1");
  lcd.setCursor(2,1);
  lcd.print("Rec L  R 0/1");
  if(digitalRead(tombol4)==0)
  {
     delay(150); lcd.clear();tone(255,150);
     lcd.setCursor(0,0);
     lcd.print("Read Right Sens");
     delay(1000);
     lcd.clear();
     while(1)
     {
      readRightSensor();
     }
  } 
/*
//### cara record data
1. taruh robot di bagian background lintasan
2. tekan tombol record data
3. geser robot secara perlahan menuju garis lintasan sampai seluruh sensor 
   garis telah mengenai garis lintasan
//### cara melihat apakah robot telah bisa membedakan garis dengan menekan tombol 
      convert logic/ 0/1 pada menu di atas, apabila sensor mengenai garis maka pembacaan pada lcd akan
      menampilkan emoticon block sedangkan ketika berada di background akan membaca setengah block
*/      
  if(digitalRead(tombol2)==0)
  {
      delay(150); lcd.clear();tone(255,150);
      lcd.setCursor(0,0);
      lcd.print("Record Data");
      delay(1000);
      lcd.clear();
      actifMux();
      recordData();
      averageLine();
      lcd.clear();
  }
  if(digitalRead(tombol3)==0)
  {
      delay(150); lcd.clear();tone(255,150);
      lcd.setCursor(0,0);
      lcd.print("Convert Logic");
      delay(1000);
      lcd.clear();
      while(1)
      {
        convertLogic(); 
      }
  } 
  if(digitalRead(tombol1)==0)
  {    
      delay(150); lcd.clear();tone(255,150);
      lcd.setCursor(0,0);
      lcd.print("Read Left Sens");
      delay(1000);
      lcd.clear();
      while(1)
      {
        readLeftSensor();
      }
  }    
  goto setSens;
//=======================================//
//### pembacaan voltase battery yang digunakan, maksimal batteri 12V
battery:
  readADC = analogRead(batteryMonitor);
  vout = (readADC/1024)*13;
  valueBattery = (vout/13)*100;
  karakterBattery = (int)valueBattery/20;
  switch(karakterBattery)
  {
    case 0: lcd.setCursor(15,1);lcd.write(byte(2)); break;
    case 1: lcd.setCursor(15,1);lcd.write(byte(3)); break;
    case 2: lcd.setCursor(15,1);lcd.write(byte(4)); break;
    case 3: lcd.setCursor(15,1);lcd.write(byte(5)); break;
    case 4: lcd.setCursor(15,1);lcd.write(byte(6)); break;
    case 5: lcd.setCursor(15,1);lcd.write(byte(7)); break;
  }
  lcd.setCursor(0,0);
  lcd.print("Battery===>>>");
  lcd.setCursor(0,1);
  lcd.print(vout,1);
  lcd.print("V>>>");
  lcd.print(valueBattery,1);
  lcd.print("%");
  if(digitalRead(tombol1)==0 || digitalRead(tombol2)==0 || digitalRead(tombol1)==3 || digitalRead(tombol4)==0)
  {
      delay(150);tone(255,150);lcd.clear();goto awal;
  }
  goto battery;
//=======================================//
//### menu yang digunakan untuk menjalankan robot dengan metode free running
setPathOrCp:
    lcd.setCursor(0,0);
    lcd.print("eClear|Path|M|CP");
    if(digitalRead(tombol1)==LOW)
    {
        delay(150);lcd.clear();turnKe = EEPROM.read(intersecEeprom+intersectionKe); goto pathPlan;
    }
    if(digitalRead(tombol2)==LOW)
    {
      delay(150);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("clear EEPROM");
      for(int z=intersecEeprom+1; z<=512; z++)
      {
        lcd.setCursor(0,0);
        lcd.print("CLEAR->");
        lcd.print(z/5.12);
        lcd.print("%");
        EEPROM.write(z,0);
        delay(10);
      }
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("EEPROM clear");
      delay(2000);
    }
    if(digitalRead(tombol3)==LOW)
    {
        delay(150);lcd.clear();nilaiPerempatan = EEPROM.read(valueCekPoin+cekPoinEeprom);goto cekPoin;
    }
    if(digitalRead(tombol4)==LOW)
    {
      delay(150);lcd.clear();goto runMaze;
    }
    goto setPathOrCp; 
cekPoin:
    if(digitalRead(tombol2)==LOW){delay(150);valueCekPoin++;nilaiPerempatan = EEPROM.read(valueCekPoin+cekPoinEeprom);}
    if(digitalRead(tombol3)==LOW){delay(150);valueCekPoin--;nilaiPerempatan = EEPROM.read(valueCekPoin+cekPoinEeprom);}      
    if(digitalRead(tombol1)==LOW){delay(150);nilaiPerempatan++;}  
    if(digitalRead(tombol4)==LOW){delay(150);nilaiPerempatan--;}
    
    lcd.setCursor(0,0);
    lcd.print("CekPoin-->Inters");
    lcd.setCursor(0,1);
    sprintf(lcdBuff,"%3d %3d   %3d", valueCekPoin+400,valueCekPoin,nilaiPerempatan);
    lcd.print(lcdBuff); 
    EEPROM.write(valueCekPoin+400, nilaiPerempatan);
    goto cekPoin;
pathPlan:
  switch(turnKe)
  {
    case 0: lcd.setCursor(14,1);lcd.print("^");break;
    case 1: lcd.setCursor(14,1);lcd.print("R");break;
    case 2: lcd.setCursor(14,1);lcd.print("L");break;
    case 3: lcd.setCursor(14,1);lcd.print("X");break;
    case 4: lcd.setCursor(14,1);lcd.print("!");break;
    case 5: lcd.setCursor(14,1);lcd.print("&");break;
  }
  if(digitalRead(tombol2)==LOW){delay(150);turnKe++;lcd.clear();}
  if(digitalRead(tombol3)==LOW){delay(150);turnKe--;lcd.clear();}
  if(digitalRead(tombol1)==LOW){delay(150);intersectionKe++;lcd.clear();  turnKe = EEPROM.read(intersecEeprom+intersectionKe);}
  if(digitalRead(tombol4)==LOW){delay(150);intersectionKe--;lcd.clear();  turnKe = EEPROM.read(intersecEeprom+intersectionKe);}
  if(turnKe < 0){turnKe = 5;}if(turnKe > 5){turnKe = 0;}
  if(intersectionKe < 1 ){intersectionKe = 1;}
  
  lcd.setCursor(0,0);
  lcd.print("Intersection Ke");
  lcd.setCursor(0,1);
  sprintf(lcdBuff,"%3d %3d %3d",intersectionKe+intersecEeprom,intersectionKe,turnKe);
  lcd.print(lcdBuff);
  EEPROM.write(intersecEeprom+intersectionKe,turnKe);
goto pathPlan;
runMenu:
    lcd.setCursor(0,0);
    lcd.print("(^_^)(&_&)");
    lcd.setCursor(0,1);
    lcd.print("CP|Free|Path|E|3");
    if(digitalRead(tombol1)==LOW){delay(150);lcd.clear();goto freeRunning;}  
    if(digitalRead(tombol2)==LOW){delay(150);lcd.clear();goto startCekPoin;}
    if(digitalRead(tombol3)==LOW){delay(150);lcd.clear();goto mirror;}
    if(digitalRead(tombol4)==LOW){delay(150);lcd.clear();lcd.setCursor(0,0);lcd.print("Run Path");delay(2000);lcd.clear();awalStartBacaEeprom = intersecEeprom+1;
    goto gogogo;}    
    goto runMenu;    
startCekPoin:   
    if(digitalRead(tombol2)==LOW){startCPke++;delay(150);lcd.clear();}
    if(digitalRead(tombol3)==LOW){startCPke--;delay(150);lcd.clear();}  
    lcd.setCursor(0,0);
    sprintf(lcdBuff, "Start CP Ke:%2d",startCPke);
    lcd.print(lcdBuff);
    if(digitalRead(tombol1)==LOW)
    {
        delay(150);lcd.clear();   
        if(startCPke == 0)
        {
          awalStartBacaEeprom = intersecEeprom+1;//karena startCPke = 0 sedangkan eeprom dimulai ke 50
        }
        else
        {
          int readStartCekPoin = cekPoinEeprom+startCPke;
          awalStartBacaEeprom = EEPROM.read(readStartCekPoin)+intersecEeprom;//50 dikarenakan pembacaan di bawah sudah di tambah 1 jadi 51       
        }
        lcd.clear();
        goto gogogo;
    }
    if(digitalRead(tombol4)==LOW)
    {
        delay(150);lcd.clear();goto runMenu;
    }    
    
    goto startCekPoin;  
mirror:
    for(int i=0;i<=250;i++)
    {    
        eepromMirror = EEPROM.read(i+intersecEeprom);
        switch(eepromMirror)
        {
            case 0: mirrorIntersection = 0;break;
            case 1: mirrorIntersection = 2;break;
            case 2: mirrorIntersection = 1;break;
            case 3: mirrorIntersection = 3;break;
            case 4: mirrorIntersection = 4;break;
            case 5: mirrorIntersection = 5;break;
        }
        EEPROM.write(i+intersecEeprom, mirrorIntersection);
        lcd.setCursor(0,0);
        lcd.print("Please Wait...");
        delay(10);
    }
    goto runMenu;
gogogo:
  runPathPlanning();
goto gogogo;
freeRunning:
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Free Running");
  delay(2000);
  lcd.clear();
  while(1)
  {
    RUN();
  }
//=======================================//
runMaze:
  while(1)
  {
    runMazeLeft();
  }
  goto runMaze;
}
//### fungsi untuk menggerakkan robot kekanan, maju, kekiri dan berhenti
void TurnR(int pwmL, int pwmR)
{
  digitalWrite(dirLA, 1);digitalWrite(dirLB, 0);analogWrite(pwmKiri, pwmL);
  digitalWrite(dirRA, 1);digitalWrite(dirRB, 0);analogWrite(pwmKanan, pwmR); 
}
void forward(int pwmL, int pwmR)
{
  digitalWrite(dirLA, 0);digitalWrite(dirLB, 1);analogWrite(pwmKiri, pwmL);
  digitalWrite(dirRA, 1);digitalWrite(dirRB, 0);analogWrite(pwmKanan, pwmR); 
}
void TurnL(int pwmL, int pwmR)
{
  digitalWrite(dirLA, 0);digitalWrite(dirLB, 1);analogWrite(pwmKiri, pwmL);
  digitalWrite(dirRA, 0);digitalWrite(dirRB, 1);analogWrite(pwmKanan, pwmR); 
}
void stopRun()
{
  digitalWrite(dirLA, 1);digitalWrite(dirLB, 1);analogWrite(pwmKiri, 255);
  digitalWrite(dirRA, 1);digitalWrite(dirRB, 1);analogWrite(pwmKanan, 255); 
}
//=======================================//
//### fungsi yang digunakan untuk menyeleksi selektor dari multiplexer
void switchMUX(int a, int b, int c)
{
  digitalWrite(selA, a);
  digitalWrite(selB, b);
  digitalWrite(selC, c); 
}
//=======================================//
//### fungsi digunakan untuk pembacaan sensor garis dengan cara menswitching fungsi selector 
//### secara bergantian kemudian disimpan didalam sebuah variable berbentuk array
void actifMux()
{
    switchMUX(0,0,0);
    valueLine[2] = analogRead(sensLeft); 
    tombol[1] = analogRead(sensRight);
    switchMUX(0,0,1);
    valueLine[4] = analogRead(sensLeft); 
    tombol[0] = analogRead(sensRight); 
    switchMUX(0,1,0);
    valueLine[0] = analogRead(sensLeft);  
    tombol[3] = analogRead(sensRight); 
    switchMUX(0,1,1);
    valueLine[5] = analogRead(sensLeft); 
    tombol[2] = analogRead(sensRight);
    switchMUX(1,0,0);
    valueLine[1] = analogRead(sensLeft);  
    valueLine[10] = analogRead(sensRight); 
    switchMUX(1,0,1);
    valueLine[7] = analogRead(sensLeft);
    valueLine[8] = analogRead(sensRight);
    switchMUX(1,1,0);
    valueLine[3] = analogRead(sensLeft);  
    valueLine[11] = analogRead(sensRight);
    switchMUX(1,1,1);
    valueLine[6] = analogRead(sensLeft); 
    valueLine[9] = analogRead(sensRight);
}
//=======================================//
//### fungsi digunakan untuk mengaktifkan nada buzzer
void tone(int note, int timeNote)
{
  analogWrite(buzzer, ~note);
  delay(timeNote);
  analogWrite(buzzer, ~0);
}
//=======================================//
//### fungsi digunakan untuk merecord sensor garis 
void recordData()
{ 
  lcd.setCursor(0,0);
  lcd.print("Record Data");
  for(int i=0;i<=11;i++)
  {
    minSementara[i] = 1024;
    maxSementara[i] = 0;
  }
  while(ulang<=100)
  {
    for(int x=0;x<=11;x++)
    {  
      actifMux();
      minSensor[x]=min(valueLine[x],minSementara[x]);
      minSementara[x] = minSensor[x];
      maxSensor[x] = max(valueLine[x], maxSementara[x]);
      maxSementara[x] = maxSensor[x];
      eeprom_write_word((uint16_t*)(x*2)+minSensEeprom, minSensor[x]);
      eeprom_write_word((uint16_t*)(x*2)+maxSensEeprom, maxSensor[x]);
      
    }
    ulang++;
  }
 
  lcd.clear();
}
//=======================================//
//### fungsi untuk mencari nilai tengah dari garis dan background lintasan
void averageLine()
{
 for(int i=0; i<=11; i++)
  {
    lcd.setCursor(0,0);
    lcd.print(i);
    minSensor[i] = eeprom_read_word((uint16_t*)(i*2)+minSensEeprom);
    maxSensor[i] = eeprom_read_word((uint16_t*)(i*2)+maxSensEeprom);
    eAverage[i]=(maxSensor[i]+minSensor[i])/2;
    eeprom_write_word((uint16_t*)(i*2)+averageEeprom,eAverage[i]);
  }
}
//=======================================//
//### fungsi untuk menampilkan nilai sensor garis ke LCD
void readRightSensor()
{
  actifMux();
  lcd.setCursor(0,0);
  sprintf(lcdBuff, "%3d %3d %3d %3d", valueLine[8],valueLine[9],valueLine[10],valueLine[11]);
  lcd.print(lcdBuff);
  lcd.setCursor(0,1);
  sprintf(lcdBuff, "%3d %3d %3d %3d", tombol[0],tombol[1],tombol[2],tombol[3]);
  lcd.print(lcdBuff);
}
void readLeftSensor()
{
  actifMux();
  lcd.setCursor(0,0);
  sprintf(lcdBuff, "%3d %3d %3d %3d", valueLine[0],valueLine[1],valueLine[2],valueLine[3]);
  lcd.print(lcdBuff);
  lcd.setCursor(0,1);
  sprintf(lcdBuff, "%3d %3d %3d %3d", valueLine[4],valueLine[5], valueLine[6],valueLine[7]);
  lcd.print(lcdBuff);
}
//=======================================//
//### fungsi untuk membedakan mana yang garis dan mana yang background lintasan
void convertLogic()
{
  for(int i=0; i<=11; i++)
  {
    actifMux();
    if(gantiBackground == 0)
    {
      if(valueLine[i]<eeprom_read_word((uint16_t*)(i*2)+averageEeprom))
      {
        binerLine[11-i] = 1;
        lcd.setCursor(2+i,1);
        lcd.write(byte(1)); 
      }
      else
      {
        binerLine[11-i] = 0;
        lcd.setCursor(2+i,1);
        lcd.write(byte(0)); 
      }
    }
    else
    {
      if(valueLine[i]<eeprom_read_word((uint16_t*)(i*2)+averageEeprom))
      {
        binerLine[11-i] = 0;
        lcd.setCursor(2+i,1);
        lcd.write(byte(1)); 
      }
      else
      {
        binerLine[11-i] = 1;
        lcd.setCursor(2+i,1);
        lcd.write(byte(0)); 
      }
    }
    
  }
}
//=======================================//
//### fungsi untuk mengubah dari variable array menjadi satu buah variable bernilai integer
void convertBiner()
{
  convertLogic();
  hexaSensor = ((binerLine[1]*1)+(binerLine[2]*2)+(binerLine[3]*4)+(binerLine[4]*8)+(binerLine[5]*16)
                +(binerLine[6]*32)+(binerLine[7]*64)+(binerLine[8]*128)+(binerLine[9]*256)+(binerLine[10]*512));
}
//=======================================//
//### fungsi untuk menjalankan robot
void RUN()
{
//### mengambil nilai dari register EEPROM
  Kp = EEPROM.read(kpEeprom); 
  Kd = EEPROM.read(kdEeprom);
  Ki = EEPROM.read(kiEeprom);
  Ts = EEPROM.read(tsEeprom);
  max_pwm_ref = EEPROM.read(rpmEeprom);
//=======================================//
//### alur pembacaan sensor garis oleh robot dengan memberikan niai error pada settiap pembacaan
//### angka 1 menandakan sensor garis membaca garis dari lintasan, sedangkan yg 0 background lintasan
  convertBiner();
  switch(hexaSensor)
  {
    case 0b0000000001: error = 9;break;//---->sensor sebelah kanan
    case 0b0000000011: error = 8;break;
    case 0b0000000010: error = 7;break;
    case 0b0000000110: error = 6;break;
    case 0b0000000100: error = 5;break;
    case 0b0000001100: error = 4;break;
    case 0b0000001000: error = 3;break;
    case 0b0000011000: error = 2;break;
    case 0b0000010000: error = 1;break;   
    case 0b0000110000: error = 0;break;
    case 0b0000100000: error = -1;break;
    case 0b0001100000: error = -2;break;
    case 0b0001000000: error = -3;break;
    case 0b0011000000: error = -4;break;
    case 0b0010000000: error = -5;break;
    case 0b0110000000: error = -6;break;
    case 0b0100000000: error = -7;break;
    case 0b1100000000: error = -8;break;
    case 0b1000000000: error = -9;break;//---->sensor sebelah kiri
    case 0b0000000000:
      if(error>=7)
      {
        error = 10;
      }
      if(error<=-7)
      {
        error = -10;
      }
    break; 
    default :
      for(int i=1; i<=3; i++)
      {
        for(int j=8; j<=10; j++)
        {
          if(binerLine[4-i] == 1 && binerLine[j] == 1)
          { 
            stopRun();
            tandaT=2;
          }
        }
      }
      for(int i=4;i<=7;i++)
      {
        if(binerLine[i] == 1 && binerLine[1] == 1)//sebelah kanan
        {
            stopRun();
            tandaL = 1;       
        }
        if(binerLine[i] == 1 && binerLine[10] == 1)//sebelah kiri
        {
            stopRun();
            tandaL = 2;
        } 
      }
    break;
  }
  Max_MV = Kp*9;      //pembatasan nilai output dari hasil PID
//### rumus PID digital
  P = Kp * error;
        
  D1 = Kd*8;                         
  D2 = D1 / Ts;
  D3 = error - last_error;
  D = D2 * D3;

  I1 = Ki/8;
  I2 = error + last_error;
  I3 = I1 * I2;
  I = I3 * Ts;

  last_error = error;

  Pd = P + D;
  MV = Pd + I;  
//=======================================//  
  if(MV>=-Max_MV && MV<=Max_MV)         //jika output PID tidak melebihi nilai pembatasan output PID maka robot akan berjalan
  {                                     //maju dengan pengaturan PWM kanan dan kiri
      pid_l  = max_pwm_ref + MV; 
      pid_r  = max_pwm_ref - MV; 
                          
      if (pid_l < 0) pid_l = 0;
      if (pid_l > 255) pid_l = 255;
      if (pid_r < 0) pid_r = 0;
      if (pid_r > 255) pid_r = 255;
      forward(pid_r,pid_l);  
  }  
  else if(MV<-Max_MV)                   //jika output PID lebih kecil dari nilai batasan output PID maka robot akan putar KANAN
  { 
     TurnL(200,100);
  }   
  else if(MV>Max_MV)                    //jika output PID lebih besar dari nilai batasan output PID maka robot akan putar KIRI
  {
     TurnR(100,200); 
  }
  else
  {
     forward(pid_r,pid_l);
  } 
  lcd.setCursor(0,0);
  sprintf(lcdBuff, "%3d %3d %2d",pid_l,pid_r,error);
  lcd.print(lcdBuff);
}
void runPathPlanning()
{
jalan:
  RUN();
  
  if((binerLine[0] == 1 || binerLine[11] == 1) && (hexaSensor != 0b0000000000 || tandaT == 2))//salah satu sayap hitam, depan baca hitam
   {                                                                                          //atau tandaT = 2
    while(1)
    {
      turnKe = EEPROM.read(intersectionKe+awalStartBacaEeprom-1);
      switch(turnKe)
      {
        case 0: 
lurus:    
            for(int xq = 0; xq<=3; xq++)
            {  
                convertBiner();
                while(binerLine[3-xq] == 1 || binerLine[xq+8] == 1)
                {
                    RUN();
                }
            } 
            
            /*
            while(1)
            {
              forward(100,100);
              convertBiner();
              if(binerLine[0] == 1 || binerLine[11] == 1)
              {
                stopRun();
                goto bawah;                            
              }
            }
            bawah:
            */
            intersectionKe++;
            tandaT = tandaL = 0;
            goto jalan; break;
        case 1: 
belokR90: 
          lcd.clear();stopRun();
          //lcd.setCursor(0,1);
          //for(int xy=4; xy>1; xy--)
          for(int xy=1; xy<4; xy++)
          {     
            while(binerLine[xy] != 1)
            {  
              convertLogic();
              TurnR(85,85);
            }
            lcd.setCursor(0,0);
            lcd.print(xy);
            
          }
          /*
          while(binerLine[10] != 1)
          {
            convertLogic();
            TurnR(85,85);
          }
          while(binerLine[8] != 1)
          {
            convertLogic();
            TurnR(85,85);
          }
          */
          stopRun();
          intersectionKe++;
          tandaT = 0;
          tandaL = 0;
          goto jalan; 
        break;
        case 2: 
belokL90:
          lcd.clear();stopRun();
          lcd.setCursor(0,1);
          //for(int yx=1; yx<4; yx++)
          for(int yx=4; yx>1; yx--)
          {
            while(binerLine[6+yx] != 1)
            {   
              convertLogic();
              TurnL(85,85);
            }
            lcd.setCursor(0,0);
            lcd.print(yx);
          }
          
          /*
          while(binerLine[1] != 1)
          {
            convertLogic();
            TurnL(85,85);
          }
          while(binerLine[3] != 1)
          {
            convertLogic();
            TurnL(85,85);
          }
          */
          stopRun();
          intersectionKe++;
          tandaT = 0;
          tandaL = 0;
          goto jalan; 
        break;
        case 3:
          lcd.clear();stopRun();
          lcd.setCursor(0,0);
          lcd.print("Finish");
        break;
        case 4:
          lcd.clear();
          gantiBackground = 1;
          while(countBlackWhite<=100)
          {   
              forward(100,100);
              countBlackWhite++;
          }
          goto lurus;    
        break;
        case 5:
          lcd.clear();
          gantiBackground = 0;
          while(countBlackWhite<=100)
          {          
              forward(100,100);
              countBlackWhite++;
          }
          goto lurus;
        break; 
      }
    }
  }
  if(binerLine[0] == 1 && tandaL == 2 && turnKe == 2)//sayap kiri baca garis, tandaL=2 liat run, belok ke kiri
  {   
    while(binerLine[0] == 1)//selama sayap kiri baca garis hitam, maka...
    {
        RUN();
    }         
    while(binerLine[0] == 0)//jika sudah membaca background,maka...
    {
        convertBiner();
        if(hexaSensor != 0b0000000000)//jika sensor depan membaca adanya garis, maka
        {
           stopRun();
           goto belokL90; 
        }
        else
        {
            goto belokLNormal;    
        }
    }         
  }
  if(binerLine[11] == 1 && tandaL == 1 && turnKe == 0)//sayap kanan baca garis tetapi instruksi tetep lurus
  {            
    while(binerLine[11] == 1)
    {
        RUN();
    } 
    while(binerLine[11] == 0)
    {
        convertBiner();
        if(hexaSensor != 0b0000000000)
        {
           stopRun();
           goto jalan; 
        }
        else
        {
            goto belokRNormal;    
        }
    }        
    
  }
  if(binerLine[0] == 1 && tandaL == 2 && turnKe == 0)//sayap kiri baca garis hitam dan instruksi lurus
  {   
    while(binerLine[0] == 1)
    {
        RUN();
    }         
    while(binerLine[0] == 0)
    {
        convertBiner();
        if(hexaSensor != 0b0000000000)
        {
           stopRun();
           goto jalan; 
        }
        else
        {
            goto belokLNormal;    
        }
    }         
  }
  if(binerLine[11] == 1 && tandaL == 1 && turnKe == 1)//sayap kanan baca garis dan instruksi belok kanan
  {            
    while(binerLine[11] == 1)
    {
        RUN();
    } 
    while(binerLine[11] == 0)
    {
        convertBiner();
        if(hexaSensor != 0b0000000000)
        {
           stopRun();
           goto belokR90; 
        }
        else
        {
            goto belokRNormal;    
        }
    }        
    
  }
  if(binerLine[11] == 1 && tandaL == 1 && turnKe == 0)
  {      
    goto lurus;
  } 
  if(binerLine[0] == 1 && tandaL == 2 && turnKe == 0)
  { 
    goto lurus;
  }
  if(binerLine[0] == 1 && hexaSensor == 0b0000000000 && tandaT == 0)//jika sayap kiri baca garis, sensor depan background dan tanda tidak ada
  {                                                                 //belok normal belokan berbentuk 90`  
belokLNormal:
    stopRun();
    for(int xz=4; xz>1; xz--)
    {
      convertLogic();
      while(binerLine[6+xz] != 1)
      {
        convertLogic();
        TurnL(100,200);
      }       
    }
    stopRun(); 
    tandaT = tandaL = 0;
  }
  if(binerLine[11] == 1 && hexaSensor == 0b0000000000 && tandaT == 0)//90` belokan yang ke kanan
  {   
belokRNormal:
    stopRun();
    for(int zx=1; zx<4; zx++)
    {
      convertLogic();
      while(binerLine[zx] != 1)
      {
        convertLogic();
        TurnR(200,100);
      }
    }
    stopRun();
    tandaT = tandaL = 0;
  }                 
}

/*
 * 0 = lurus
 * 1 = kanan
 * 2 = kiri
 * 3 = putar balik
 */
void runMazeLeft()
{
  RUN();
  if(digitalRead(tombol1)==LOW)
  {
    stopRun();
    lcd.clear();
    while(1)
    {
      for(int i=1; i<=4; i++)
      {
        lcd.setCursor(0,0);
        lcd.print(i);
        lcd.setCursor(0,1);
        lcd.print(EEPROM.read(intersecEeprom+i));
        delay(2000);
        lcd.clear();
      }
    }
  }
  if(hexaSensor == 0b1111111111 && binerLine[11] == 1 && binerLine[0] == 1)
  {
    while(1)
    {
      runPathPlanning(); 
    }
  }
  if((tandaT == 2 || tandaL == 2) && binerLine[11] == 1)
  {
    while(1)
    {
      RUN();
      if((tandaT == 2 || tandaL == 2) && binerLine[11] != 1)
      {
        unsigned char lastEeprom = EEPROM.read(intersecEeprom+intersectionKe - 1);
        if(lastEeprom == 3)
        {
          konversiMazeL();
        }
        else
        {
          EEPROM.write(intersecEeprom+intersectionKe, 2);
          /*
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print(EEPROM.read(intersecEeprom+intersectionKe));
          lcd.setCursor(0,1);
          lcd.print(intersecEeprom+intersectionKe);
          delay(2000);
          lcd.clear();
          */
          turnLmaze();
        }
        break;
      }
    }
  }
  if(tandaL == 1 && binerLine[0] == 1)
  {
    while(1)
    {
      RUN();      
      if(binerLine[0] != 1)
      {
        unsigned char lastEeprom = EEPROM.read(intersecEeprom+intersectionKe - 1);
        if(lastEeprom == 3)
        {
          konversiMazeS();
        }
        else
        {
          EEPROM.write(intersecEeprom+intersectionKe, 0); 
          /*
          lcd.clear();
          lcd.setCursor(2,0);
          lcd.print(EEPROM.read(intersecEeprom+intersectionKe));
          lcd.setCursor(2,1);
          lcd.print(intersecEeprom+intersectionKe);
          delay(2000);
         */
          intersectionKe++;
          tandaT = 0;
          tandaL = 0;  
        }
        break;
      }
    }
  }
  if(hexaSensor == 0b0000000000 && binerLine[0] == 0 && binerLine[11] == 0 
      && (error >= -4 && error <= 4) && tandaL == 0 && tandaT == 0)
  {
    EEPROM.write(intersecEeprom+intersectionKe, 3);
    /*
    lcd.clear();
    lcd.setCursor(4,0);
    lcd.print(EEPROM.read(intersecEeprom+intersectionKe));
    lcd.setCursor(4,1);
    lcd.print(intersecEeprom+intersectionKe);
    delay(2000);
    */
    turnLmaze();
  }
}
void konversiMazeL()
{
  unsigned char secondLastEeprom = EEPROM.read(intersecEeprom+intersectionKe - 2);
  switch(secondLastEeprom)
  {
    case 0: 
            /*
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print(EEPROM.read(intersecEeprom+intersectionKe - 2));
            lcd.setCursor(10,0);
            lcd.print(intersecEeprom+intersectionKe);
            */
            EEPROM.write(intersecEeprom+intersectionKe - 2, 1);
            intersectionKe -=2;
            tandaT = tandaL = 0;
            /*
            lcd.setCursor(0,1);
            lcd.print(EEPROM.read(intersecEeprom+intersectionKe));
            lcd.setCursor(10,1);
            lcd.print(intersecEeprom+intersectionKe);
            delay(10000);
            */
            intersectionKe++;
            break;
            
    case 1: 
            /*
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print(EEPROM.read(intersecEeprom+intersectionKe - 2));
            lcd.setCursor(10,0);
            lcd.print(intersecEeprom+intersectionKe);
            */
            
            EEPROM.write(intersecEeprom+intersectionKe - 2, 3);
            intersectionKe -=2;
            tandaT = tandaL = 0;

            /*
            lcd.setCursor(0,1);
            lcd.print(EEPROM.read(intersecEeprom+intersectionKe));
            lcd.setCursor(10,1);
            lcd.print(intersecEeprom+intersectionKe);
            delay(10000);
            */
            intersectionKe++;
            break;
            
    case 2: 
            /*
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print(EEPROM.read(intersecEeprom+intersectionKe - 2));
            lcd.setCursor(10,0);
            lcd.print(intersecEeprom+intersectionKe);
            */
            EEPROM.write(intersecEeprom+intersectionKe - 2, 0);
            intersectionKe -=2;
            tandaT = tandaL = 0;
            /*
            lcd.setCursor(0,1);
            lcd.print(EEPROM.read(intersecEeprom+intersectionKe));
            lcd.setCursor(10,1);
            lcd.print(intersecEeprom+intersectionKe);
            delay(10000);
            */
            intersectionKe++;
            break;
  }
}
void konversiMazeS()
{
  unsigned char secondLastEeprom = EEPROM.read(intersecEeprom+intersectionKe - 2);
  switch(secondLastEeprom)
  {
    case 2: 
            /*
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print(EEPROM.read(intersecEeprom+intersectionKe - 2));
            lcd.setCursor(10,0);
            lcd.print(intersecEeprom+intersectionKe);
            */
            EEPROM.write(intersecEeprom+intersectionKe - 2, 1);
            intersectionKe -=2;
            tandaT = tandaL = 0;
            /*
            lcd.setCursor(0,1);
            lcd.print(EEPROM.read(intersecEeprom+intersectionKe));
            lcd.setCursor(10,1);
            lcd.print(intersecEeprom+intersectionKe);
            delay(10000);
            */
            intersectionKe++;
            break;
  }
}
void turnRmaze()
{
  stopRun();
  for(int xy=1; xy<4; xy++)
  {     
    while(binerLine[xy] != 1)
    {  
      convertLogic();
      TurnR(85,85);
    }    
  }   
  stopRun();
  intersectionKe++;
  tandaT = 0;
  tandaL = 0;
}
void turnLmaze()
{
  stopRun();
  for(int yx=4; yx>1; yx--)
  {
    while(binerLine[6+yx] != 1)
    {   
      convertLogic();
      TurnL(85,85);
    }
  }
  stopRun();
  intersectionKe++;
  tandaT = 0;
  tandaL = 0;
}

