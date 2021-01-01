
#include <QMC5883LCompass.h>
#include "Ublox.h"
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Servo.h>
#include <LoRa.h>
#include "PWM.hpp"

const int chipSelect = BUILTIN_SDCARD;

char text[70];

float t2 = 0; 
float t1 = 0; 
int count = 1; 
char namebuff[20]; 
unsigned int addr = 0;
String filename; 
int led = 13;
int datatest = 0; 

float ail1 = 0;
float ail2 = 0;

float elv = 0; 


char magn_text[30];
char lat_text[30];
char lon_text[30];
char alt_text[30];
char cog_text[30];
char spe_text[30];
char sat_text[30];
char pwm1_text[30];
char pwm2_text[30];
char pwm3_text[30];


float B = 0; 
float cog = 0; 
float spe = 0; 
float magn = 0; 
float bearing = 0; 
float degr = 0;
float offst = 0; 
float cmd = 0; 
float alt = 0; 
float lata = 0; 
float latb = 45.195889269053446; 
float lona = 0; 
float lonb = 5.68393349647522; 
float sat = 0; 
int toss = 0; 

int counter = 0;
void getgps(); 
void getcompass(); 

PWM ch1(4); 
PWM ch2(5); 
PWM ch3(6); 

int pwm1 = 0; 
int pwm2 = 0; 
int pwm3 = 0;


Servo servo1;
Servo servo2;
Ublox M8_Gps;
QMC5883LCompass compass;

void setup() {

  ch1.begin(true); // ch1 on pin 2 reading PWM HIGH duration
  ch2.begin(true); // ch2 on pin 3 reading PWM HIGH duration
  ch3.begin(true); // ch3 on pin 18 reading PWM HIGH duration
  
  pinMode(led, OUTPUT);
  
  //eppclear()
  epprom(); 
  

  Serial.begin(57600);
  SD.begin(chipSelect);
  Serial7.begin(57600);
  compass.init();
  compass.setCalibration(-1602, 1045, -1826, 647, -2963, 0);
  LoRa.begin(433E6);
  servo1.attach(19);
  servo2.attach(22);
  

}

void loop() {

  t1 = millis(); 
  
  if (t1-t2>500) {
     
    t2 = millis();
    
      
      datalog(); 

      if (!LoRa.begin(433E6)) { 
        
      }

      else { 
        
      sendtelem(); 
      
      }
      
      
  }  


    getgps(); 
    getcompass();

    //B = magn; 
  
    //comptobj(); 
    rcread(); 
    servocmd();
    
    
 
}
//-----------------------------------------------//
void datalog() { 
  
  File dataFile = SD.open(namebuff, FILE_WRITE);

    if (dataFile) {
      
      dtostrf(magn, 4, 1, magn_text);
      dtostrf(lata, 10, 10, lat_text);
      dtostrf(lona, 10, 10, lon_text);
      dtostrf(alt, 4, 1, alt_text);
      dtostrf(cog, 4, 1, cog_text);
      dtostrf(spe, 5, 1, spe_text);
      dtostrf(sat, 3, 1, sat_text);
      dtostrf(pwm1, 4, 1, pwm1_text);
      dtostrf(pwm2, 4, 1, pwm2_text);
      dtostrf(pwm3, 4, 1, pwm3_text);

      snprintf(text, 80, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s", magn_text, lat_text, lon_text, alt_text, cog_text, spe_text, sat_text, pwm1_text, pwm2_text, pwm3_text);
      
      dataFile.println(text);
      dataFile.close();
 
      
    }  
    
}
//-----------------------------------------------//
void epprom() {
  
  EEPROM.get(addr, count);
  sprintf(namebuff, "Data%0d.txt", count);
  EEPROM.write(addr, (count+1)); 
}
//-----------------------------------------------//
void eppclear() { 
  for ( unsigned int i = 0 ; i < EEPROM.length() ; i++ )
  EEPROM.write(i, 0); 
}
//-----------------------------------------------//
void getgps() { 
if(!Serial7.available())
 return;

  while(Serial7.available()){
    
         char c = Serial7.read();
         if (M8_Gps.encode(c)) {

           cog = ((M8_Gps.course)/100); 
           spe = (M8_Gps.speed)/100;
           lata = M8_Gps.latitude; 
           lona = M8_Gps.longitude; 
           alt = M8_Gps.altitude; 
           sat = M8_Gps.sats_in_use;
           
  }
 } 
}
//-----------------------------------------------//
void getcompass() { 
  compass.read(); 
  magn = 360 - ( ((compass.getAzimuth())+90) % 360 ) ;
}
//-----------------------------------------------//
void averagedir() { 
  if ((sqrt((magn-cog)*(magn-cog))) < 180) { 
    
    B = (magn + cog) / 2 ;   
    
  }

  else { 

    B = fmod((((magn + cog)/2)+180), 360);
    
  } 
}
//-----------------------------------------------//
void comptobj() { 
  
   float DX = latb - lata;
   float DY = lonb - lona;
   float rad = atan2(DY, DX);
   degr = rad * (180 / PI);
   bearing = fmod(degr + 360, 360); 
   float A = bearing ;
   // float B = magn ;
   
    if (sqrt((A-B)*(A-B)) < 180) { 
      offst = (A-B);
    }

    else { 
    
      if ((A-B) < 0) {
        offst = 360 +(A-B);
      }

      else { 
       offst = (-360) + (A-B) ;
  }
 }
}
//-----------------------------------------------//
void servocmd() { 
   //cmd = map(offst, -180,180, 1000, 2000); 
   //servo.write(cmd);
   
    
   
   float diff1 = sqrt(pow(1500-pwm1, 2)) ; 
   float diff2 = sqrt(pow(1500-pwm2, 2)) ; 

   if (1500-pwm1 > 0) { 

   ail1 = 1500 - diff1 ;
   ail2 = 1500 + diff1 ;
    
   }
   else { 

   ail1 = 1500 + diff1 ;
   ail2 = 1500 - diff1 ;
    
   }

   if (1500-pwm2 > 0) { 

   elv = 1500 + diff2 ;
    
   }
   else { 

   elv = 1500 - diff2 ;
    
   }
   

   //Serial.print(ail1); Serial.print(" ");
   //Serial.println(ail2);

   float cmd1 = (0.5 * ail1) + (0.5* elv);
   float cmd2 = (0.5 * ail2) + (0.5* elv); 

   if (cmd1 > 2000 or cmd2 > 2000 or cmd1 < 1000 or cmd2 < 1000) { 
    cmd1 = 1500; 
    cmd2 = 1500; 
   }

   Serial.print(cmd1); Serial.print(" ");
   Serial.println(cmd2);
   
   servo1.write(cmd1); 
   servo2.write(cmd2); 
   
   
}
//-----------------------------------------------//
void sendtelem() { 
  
      //dtostrf(magn, 4, 1, magn_text);
      dtostrf(lata, 10, 10, lat_text);
      dtostrf(lona, 10, 10, lon_text);
      dtostrf(alt, 3, 1, alt_text);
      //dtostrf(cog, 4, 1, cog_text);
      //dtostrf(spe, 5, 1, spe_text);
      dtostrf(sat, 3, 1, sat_text);

      snprintf(text, 45, "%s,%s,%s,%s", lat_text, lon_text, alt_text, sat_text);
      //Serial.println(text); 
      
 
      
      LoRa.beginPacket();
      LoRa.println(text);
      LoRa.endPacket();  
         
}
//-----------------------------------------------//
void rcread() { 
  
  pwm1 = ch1.getValue();
  pwm2 = ch2.getValue();
  pwm3 = ch3.getValue();
  //Serial.print("CH1 : ");
  //Serial.print(pwm1); Serial.print(" ");
  //Serial.print("CH2 : ");
  //Serial.print(pwm2); Serial.print(" ");
  //Serial.print("CH3 : ");
  //Serial.println(pwm3); 
  
}
