 
//Prateek
//www.prateeks.in

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27 ,16,2);

#include "Countimer.h"
Countimer tdown;
#include <EEPROM.h>

#define bt_set    A0
#define bt_up     A1
#define bt_down   A2
#define bt_start  A3

int time_s = 0;
int time_m = 0;

int set = 0;
int flag1=0, flag2=0;

int relay = 8;

const int buttonPin = 2;
const int relayPin = 12;
int buttonState = 0;

void setup() {
  lcd.init();
  lcd.backlight();
  Serial.begin (9600);
  pinMode(relayPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(bt_set,   INPUT_PULLUP);
  pinMode(bt_up,    INPUT_PULLUP);
  pinMode(bt_down,  INPUT_PULLUP);
  pinMode(bt_start, INPUT_PULLUP);
  
  pinMode(relay, OUTPUT);
  
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("  UVC SANTIZER ");
  delay(1000);
  lcd.setCursor(0,1);
  lcd.print("  STARTING... ");
  tdown.setInterval(print_time, 999);
  eeprom_read();
  delay(1000);
  lcd.clear();
}

void print_time(){
time_s = time_s-1;
if(time_s<0){time_s=59; time_m = time_m-1;}
}

void tdownComplete(){Serial.print("ok");}

//tdown.stop(); 

void loop(){
tdown.run();
if ((time_m<=0)&&(time_s<=0)){
   tdown.stop();
   digitalWrite(relayPin,HIGH);
   digitalWrite(relay,LOW);
}else{
  digitalWrite(relay,LOW);
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    digitalWrite(relayPin, HIGH);
    digitalWrite(relay,HIGH);
    tdown.start();
    } else {
      digitalWrite(relayPin, LOW);
      digitalWrite(relay,LOW);
      tdown.pause();
    }

  }

if(digitalRead (bt_set) == 0){
if(flag1==0 && flag2==0){flag1=1;
set = set+1;
if(set>3){set=0;}
delay(100); 
}
}else{flag1=0;}

if(digitalRead (bt_up) == 0){
if(set==0){tdown.start(); flag2=1;}
if(set==1){time_s++;}
if(set==2){time_m++;}
if(time_s>59){time_s=0;}
if(time_m>10){time_m=0;}
if(set>0){eeprom_write();}
delay(200); 
}

if(digitalRead (bt_down) == 0){
if(set==0){tdown.stop(); flag2=0;}
if(set==1){time_s--;}
if(set==2){time_m--;}
if(time_s<0){time_s=59;}
if(time_m<0){time_m=10;}
if(set>0){eeprom_write();}
delay(200); 
}

if(digitalRead (bt_start) == 0){ flag2=1; 
  eeprom_read(); 
  digitalWrite(relay, HIGH); 
  tdown.restart(); 
  tdown.start();
}

lcd.setCursor(0,0);
if(set==0){lcd.print("     Timer     ");}
if(set==1){lcd.print("Set Timer SS  ");}
if(set==2){lcd.print("Set Timer MM  ");}

lcd.setCursor(4,1);
if(time_m<=9){lcd.print(" 0");}
lcd.print(time_m);
lcd.print(":");
if(time_s<=9){lcd.print("0");}
lcd.print(time_s);
lcd.print("   ");

if(time_s==0 && time_m==0 && flag2==1){flag2=0;
tdown.stop(); 
digitalWrite(relay, LOW);
delay(300);
}

if(flag2==1){digitalWrite(relay, HIGH);}
else{digitalWrite(relay, LOW);}

delay(1);
}

void eeprom_write(){
EEPROM.write(1, time_s);  
EEPROM.write(2, time_m); 
}

void eeprom_read(){
time_s =  EEPROM.read(1);
time_m =  EEPROM.read(2);

}
