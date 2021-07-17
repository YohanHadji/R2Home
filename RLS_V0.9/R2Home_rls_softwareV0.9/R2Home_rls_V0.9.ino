#include <TinyGPS++.h>
#include "Adafruit_BMP280.h"
#include "QMC5883LCompass.h"
#include <SD.h>
#include <EEPROM.h>
#include <SBUS.h>
#include <EasyBuzzer.h>
#include <Watchdog.h>
#include <PWMServo.h>
#include <movingAvg.h>


#include <Adafruit_NeoPixel.h>
#define LED_PIN     3
#define LED_COUNT  1
#define BRIGHTNESS 15
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800); 

int duration = 0; 
int timeled = 0; 
int lastled = 0; 


Adafruit_BMP280 bmp;
TinyGPSPlus gps;
TinyGPSCustom fixtype(gps, "GNGSA", 2);
File dataFile; 
SBUS rx(Serial1);
QMC5883LCompass compass;
Watchdog watchdog;
PWMServo left;
PWMServo right;
PWMServo deploy; 
movingAvg voltage(50);    
movingAvg s_left(50);  
movingAvg s_right(50);  
movingAvg vs(5); 

// RX // 
uint16_t channels[16];
bool failSafe;
bool lostFrame;

// SD CARD // 
const int chipSelect = BUILTIN_SDCARD;
char namebuff[20]; 
unsigned int addr = 0;
String filename; 
int datatest = 0; 
int count = 0; 

char lat_text[30];
char lon_text[30];
char altgps_text[30];
char cog_text[30];
char magn_text[30]; 
char speed_text[30];
char sat_text[30];
char fixtype_text[10];
char hdop_text[10];
char posage_text[10]; 
char count_text[10]; 
char pwrontime_text[10];

char altbaro_text[30];
char vspeed_text[30];
char vtimespeed_text[30];

char altmerged_text[30];
char barow_text[30];
char gpsw_text[30]; 
char bearing_text[30]; 
char latb_text[30];
char lonb_text[30];
char offst_text[30]; 
char windspeed_text[30]; 
char windir_text[30]; 
char mergedir_text[30]; 

char date_time[60];
char date_year[10];
char date_month[10];
char date_day[10];
char time_hour[10];
char time_minute[10];
char time_seconde[10];

char a_text[10]; 
char b_text[10]; 
char c_text[10]; 
char d_text[10]; 
char e_text[10]; 
char f_text[10]; 
char g_text[10]; 

char left_text[10]; 
char right_text[10]; 
char deploy_text[10]; 

char sdok_text[10];
char gpsok_text[10];  
char failSafe_text[10]; 
char stationary_text[10]; 
char deployed_text[10]; 
char mode_text[10]; 
char vbatt_text[10]; 
char looptime_text[10]; 

char z = 0; 

// MAIN TEXT // 
char mainSD[240];
char mainTLM[240]; 

char mintext[120];
char gps_text[120]; 
char baro_text[120];
char rc_text[120]; 
char servo_text[120]; 
char status_text[120]; 
char alt_text[120]; 
char nav_text[120]; 

// DATA // 
float lata = 0; 
float lona = 0; 
float alta = 0; 
float altgps = 0;
float altbaro = 0;
float twoDspeed = 0; 
float cog = 0; 
float sats = 0; 
float fixtyped = 0; 
float hdo = 0;
float datetime = 0;
float dateyear = 0;
float datemonth = 0;
float dateday = 0;
float timehour = 0;
float timeminute = 0;
float timeseconde = 0;
float posage = 0; 
int heading = 0; 

boolean newgps = false; 

float temperature; 
float pressure; 
float altitude = 0; 
float vspeed = 0; 
float prev_vspeed = 0; 
float prev_alt = 0; 
float vtimespeed = 0; 


float vpin = A17; 
float vbatt = 0; 


float alt1 = 0; 
float alt2 = 0; 
float alt3 = 0; 
float alt4 = 0; 


// AUTOPILOT // 
float dep_altitude = 30; 
float offset_alt =  1;
float merged_alt = 0; 

float bearing = 0; 
float offst = 0; 

float baroweight = 1;
float gpsweight = 1;

float nfs = 10; 
float windspeed = 0; 
float windir = 0; 
float fwdspeed = 0; 

int mergedir = 0; 

float latb = 0; 
float lonb = 0; 
float altb = 0; 

float servo_left = 1500;
float servo_right = 1500;
float servo_deploy = 2000;


// MISC // 
int flight_mode = 0; 
int prev_mode = 0; 
bool deployed = false; 
bool stationary = false; 
int vspeed_count = 0;
float avg = 0; 
bool initialised = false; 

bool sdok = false; 
bool gpsok = false; 

// DELAYS // 
unsigned long t1 = 0; //GPS LOOP 
unsigned long t2 = 0; //TLM
unsigned long t3 = 0; //I2C SENSOR
unsigned long t4 = 0; //BUZZER WAIT
unsigned long t5 = 0; //VSPEED
unsigned long t6 = 0; //DATA TIME
unsigned long t7 = 0; //VSPEED
unsigned long t8 = 0; //SERVO UPDATE
unsigned long t9 = 0; //VSPEED
unsigned long t10 = 0; //LOOP TIME
unsigned long t11 = 0; //STATIONARY
unsigned long t12 = 0; //LOOP TIME
unsigned long t13 = 0; //DATALOG
unsigned long t14 = 0; //BATTERY ALERT
unsigned long t15 = 0; //INIT TIMER


int loop_time = 999; 

int delaySD = 100;    // Datalog 
int delayTLM = 1000;   // Tlm 
int delay3 = 10000;  // Vtimespeed
int delay4 = 5000;  // GPS Count Buzzer Timer
float delay5 = 100;  // Vspeed 
float delay8 = 50;   // Magnetometer
float delay14 = 100; // BATTERY ALERT 
int data_time = 100;


// USER INTERFACE // 
int buzzer =   2;
float bcritical = 10.8;
float blow = 11.4;
int cam = 23; 

int m = 0; 

boolean batt_critical = false; 
boolean batt_low = false; 


// --------------------------------------------------- SETUP --------------------------------------------------- //

void setup() {

  pinMode(A17, INPUT); 
  pinMode(cam, OUTPUT);

  digitalWrite(cam, LOW); 
  
  Serial.begin(115200);
  Serial5.begin(57600); 
  
  rx.begin();
  
  gpset(115200, 10, 0, 1, 1);  

  EasyBuzzer.setPin(buzzer);

  voltage.begin();
  s_left.begin(); 
  s_right.begin(); 
  vs.begin(); 
  
  compass.init();
  //compass.setMode(0x01, 0x00, 0x00, 0xC0);
  compass.setSmoothing(25,true);
  compass.setCalibration(-360, 1956, -1179, 984, -1137, 837);  

  left.attach(6);
  right.attach(7);
  deploy.attach(8);
  
  bmp.begin(0X76);

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
  Adafruit_BMP280::STANDBY_MS_1);  

  //eppclear(); 
  epprom();  
  sdinit(); 
  
  pinMode(buzzer, OUTPUT);

  strip.begin();           
  strip.show();            
  strip.setBrightness(BRIGHTNESS);

  watchdog.enable(Watchdog::TIMEOUT_1S);

  tone(buzzer, 523); 
  delay(200); 
  tone(buzzer, 582); 
  delay(200); 
  tone(buzzer, 762); 
  delay(300); 
  noTone(buzzer); 

}

// --------------------------------------------------- LOOP --------------------------------------------------- //

void loop() {


  t10 = micros(); 
  
  getgps(); 
  datetimecmpt(); 
  geti2c(); 
  getvtimespeed(); 
  getrc(); 
  getfailsafe(); 
  getvoltage(); 
  getwindspeed(); 

  altfusion(); 
  
  flight_state(); 
  updateservo(50); 

  cmptdata();

  checkmode(); 
  getcmd(); 

  datalog();
  //Serial.println("LOOP"); 
  tlm(); 
  
  stat(); 

  watchdog.reset();
  EasyBuzzer.update();
  updateled();

  t12 = micros();

  looptime(); 
}

// --------------------------------------------------- STATE MACHINE --------------------------------------------------- // 

void flight_state() { 

  switch(flight_mode) { 
    
    case 0: 
    flight_init(); 
    break;

    case 1: 
    ready_steady(); 
    break;

    case 2: 
    flight_ascent(); 
    break;

    case 3: 
    flight_descent(); 
    break;

    case 4: 
    flight_gliding(); 
    break;

    case 5: 
    flight_gliding_auto(); 
    break;

    case 6: 
    flight_gliding_manual(); 
    break;

    case 7: 
    landed(); 
    break;

    case 8: 
    datawire(); 
    break;

    case 9: 
    datatlm(); 
    break;

  }
}

//------------------- 0 -------------------//

void flight_init() { 

  manualpilot(); 

  delaySD = 1000; 
  delayTLM = 10000; 

  if (fixtyped == 3) {
   
    EasyBuzzer.beep(3000,100,50,10,500,1);
    latb = lata ;
    lonb = lona ;
    altb = 0 ;

    offset_alt = (altgps/altitude); // Baro Initialisation  
    dep_altitude = (dep_altitude+altgps);

    strip.setBrightness(50);
    initialised = true; 
    flight_mode = 1; 
    setcam(1); 
    t15 = millis();  
  }

  if (millis()-t4>delay4) { 
    
    t4 = millis(); 
    EasyBuzzer.beep(2000,50,25,sats,100,1); 
  }
  
  setled(255, 0, 0, 25, 2000);  
}

//------------------- 1 -------------------//

void ready_steady() { 

  if (millis()-t15>1000) { 
  
  manualpilot(); 

  setled(0, 255, 0, 25, 500); 
  
  delaySD = 1000; 
  delayTLM = 5000; 
  
  if (vspeed>1) {flight_mode = 2; setled(0,0,255, 10, 100); EasyBuzzer.beep(1000,100,50,2,500,1);}
  if (vspeed<-3) {flight_mode = 3;EasyBuzzer.beep(3000,100,50,3,500,1);  strip.setBrightness(100);} 


  if (fixtyped < 2) {
    flight_mode = 0; 
  }
 } 
}

//------------------- 2 -------------------//

void flight_ascent() { 

  manualpilot(); 

  setled(0, 0, 255, 25, 250); 
  
  delaySD = 100; 
  delayTLM = 1000; 
  
  if (vspeed<0.3) {flight_mode = 1; strip.setBrightness(50);}  
  
}

//------------------- 3 -------------------//

void flight_descent() { 
  
  manualpilot(); 

  setled(255, 128, 0, 25, 1000); 

  delaySD = 100; 
  delayTLM = 1000; 

  if (vspeed>-0.3) {flight_mode = 1; strip.setBrightness(50);} 

  if (merged_alt < dep_altitude) {
    EasyBuzzer.beep(3000,100,50,5,500,1);
    wing_deployment();
    flight_mode = 4;
    strip.setBrightness(255);
    
  }
    
}

//------------------- 4 -------------------//

void flight_gliding() {

  if ((vspeed < 3) and (vspeed >-3)) { 

  if (failSafe == true) {flight_mode = 5; strip.setBrightness(100);} 
  else {flight_mode = 6; strip.setBrightness(100);}
  
  } 
  
}

//------------------- 5 -------------------//

void flight_gliding_auto() { 

  setled(255, 255, 0, 25, 1000); 
  autopilothome();
  if (failSafe == false) {flight_mode = 6;} 
  //-if (stationary == true) {flight_mode = 7; strip.setBrightness(255);}
   
}

//------------------- 6 -------------------//

void flight_gliding_manual() { 

  setled(0, 255, 255, 25, 1000); 
  if (failSafe == true) {flight_mode = 5;}
  //if (stationary == true) {flight_mode = 7; strip.setBrightness(255); setcam(0);}
  
  if (channels[4]>1000) {
    autopilotcap();
  }

  else {
    manualpilot(); 
  }
  
}

//------------------- 7 -------------------//

void landed() { 

  setled(128, 0, 255, 25, 1000); 

  setservo(1500,1500,1500);
  
  delaySD = 1000; 
  delayTLM = 5000; 

  if ((vspeed > 1) or (vspeed < -1)) {flight_mode = 1; strip.setBrightness(50);}

  if (millis()-t4>delay4) { 
    t4 = millis(); 
    EasyBuzzer.beep(4000,100,25,5,1000,3);
  }   
}

//--------------------------------------//

void wing_deployment() { 
  
  deployed = true ; 
  //Serial.println("DEPLOYED!!"); 
  //Serial5.println("DEPLOYED!!"); 
}

//--------------------------------------------------- DATA --------------------------------------------------- // 


void altfusion() { 

  if (initialised == true) { 
  
  float absvspeed = sqrt(vspeed*vspeed);
  baroweight = (1+(absvspeed/10));
  gpsweight = (sats/30);
  
  merged_alt = ((altitude*offset_alt*baroweight)+(altgps*gpsweight))/(baroweight+gpsweight);
  
 }
}

//--------------------------------------//

void cmptdata() {
  
  dtostrf(lata, 10, 10, lat_text);
  dtostrf(lona, 10, 10, lon_text);
  dtostrf(altgps, 1, 1, altgps_text);
  dtostrf(cog, 1, 0, cog_text);
  dtostrf(twoDspeed, 1, 1, speed_text);
  dtostrf(sats, 1, 0, sat_text);
  dtostrf(fixtyped, 1, 0, fixtype_text);
  dtostrf(hdo, 1, 0, hdop_text); 
  dtostrf(posage, 1, 0, posage_text);

  snprintf(gps_text, 120, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s", date_time, lat_text, lon_text, altgps_text, cog_text, speed_text, sat_text, hdop_text, posage_text, fixtype_text);

  dtostrf((altitude*offset_alt), 2, 3, altbaro_text);
  dtostrf(vspeed, 2, 2, vspeed_text);
  
  snprintf(baro_text, 120, "%s,%s", altbaro_text, vspeed_text);  

  dtostrf(merged_alt, 2, 3, altmerged_text);
  dtostrf(gpsweight, 2, 2, gpsw_text);
  dtostrf(baroweight, 2, 3, barow_text);
  dtostrf(bearing, 2, 2, bearing_text);
  dtostrf(latb, 1, 5, latb_text);
  dtostrf(lonb, 1, 5, lonb_text);
  dtostrf(offst, 1, 5, offst_text);
  dtostrf(windspeed, 1, 2, windspeed_text);
  dtostrf(windir, 1, 2, windir_text);
  dtostrf(mergedir, 1, 0, mergedir_text); 
   

  snprintf(nav_text, 120, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s", altmerged_text, barow_text, gpsw_text, bearing_text, latb_text, lonb_text, offst_text, windspeed_text, windir_text, mergedir_text);  

  dtostrf(heading, 2, 0, magn_text);

  dtostrf(channels[0], 2, 0, a_text);
  dtostrf(channels[1], 2, 0, b_text);
  dtostrf(channels[2], 2, 0, c_text);
  dtostrf(channels[3], 2, 0, d_text);
  dtostrf(channels[4], 2, 0, e_text);
  dtostrf(channels[5], 2, 0, f_text);
  dtostrf(channels[6], 2, 0, g_text);

  snprintf(rc_text, 120, "%s,%s,%s,%s,%s,%s,%s", a_text, b_text, c_text, d_text, e_text, f_text, g_text);

  dtostrf(servo_left, 2, 0, left_text);
  dtostrf(servo_right, 2, 0, right_text);
  dtostrf(servo_deploy, 2, 0, deploy_text);
  
  snprintf(servo_text, 120, "%s,%s,%s", left_text, right_text, deploy_text); 

  dtostrf(millis(), 1, 0, pwrontime_text);
  dtostrf(flight_mode, 1, 0, mode_text);
  dtostrf(gpsok, 1, 0, sdok_text);
  dtostrf(failSafe, 1, 0, failSafe_text);
  dtostrf(stationary, 1, 0, stationary_text);
  dtostrf(deployed, 1, 0, deployed_text);
  dtostrf(vbatt, 1, 3, vbatt_text);
  dtostrf(loop_time, 1, 0, looptime_text);
  
  snprintf(status_text, 60, "%s,%s,%s,%s,%s,%s,%s,%s", pwrontime_text, mode_text, sdok_text, failSafe_text, stationary_text, deployed_text, vbatt_text, looptime_text);  

  snprintf(mainSD, 240, "%s,%s,%s,%s,%s,%s,%s", status_text, gps_text, baro_text, magn_text, nav_text, rc_text, servo_text);

  snprintf(mainTLM, 240, "*%s,%s,%s,%s,%s/", status_text, gps_text, baro_text, magn_text, servo_text);
  
}

void datawire() { 

  if (millis() - t6 > data_time) { 
    t6 = millis(); 
    
    switch(z) { 

      case 'G':
      Serial.println(gps_text); 
      break; 

      case 'B':
      Serial.println(baro_text); 
      break; 

      case 'M':
      Serial.println(heading); 
      break; 

      case 'R':
      Serial.println(rc_text);
      break; 

      case 'S':
      Serial.println(servo_text); 
      break;

      case 'V':
      Serial.println(vbatt); 
      break;

      case 'A':
      Serial.println(alt_text); 
      break;

    }
  }
}

//--------------------------------------//

void datatlm() { 

  if (millis() - t6 > data_time) { 
    t6 = millis(); 
    
    switch(z) { 

      case 'G':
      Serial5.println(gps_text); 
      break; 

      case 'B':
      Serial5.println(baro_text);
      break; 

      case 'M':
      Serial5.println(heading); 
      break; 

      case 'R':
      Serial5.println(rc_text);
      break; 

      case 'S': 
      Serial5.println(servo_text);
      break;

      case 'V':
      Serial5.println(vbatt);
      break;

      case 'A':
      Serial5.println(alt_text);
      break;

    }
  }
}
//--------------------------------------//

void getvoltage() { 
  vbatt = analogRead(A17); 
  vbatt = (voltage.reading(vbatt)*0.0141796875);

  if ((vbatt < bcritical) and (vbatt > 3)) { 
    
    batt_critical = true; 
    delay14 = 100;
    
    if (millis()-t14>delay14) { 
      t14 = millis(); 
     EasyBuzzer.singleBeep(3000,25);
    }
  }

  else { 
    batt_critical = false;    
  }

  if ((vbatt < blow) and (vbatt > bcritical)) { 
    
    batt_low = true; 
    delay14 = 200;
    
    if (millis()-t14>delay14) { 
      t14 = millis(); 
      EasyBuzzer.singleBeep(3000,50);
    }
  }

  else { 
    batt_low = false; 
  }
}
//--------------------------------------//
void getwindspeed() { 

  float er = sqrt((heading-cog)*(heading-cog));
  er = ((er*71)/4068);
  float Vy = twoDspeed*sin(er);
  float AB = sqrt((Vy*Vy)+((Vy-nfs)*(Vy-nfs)));

  windir = acos((Vy-nfs)/AB);

  windir = ((windir * 4068)/71); 

  windspeed = AB; 

  fwdspeed = Vy;  
}
//--------------------------------------//

void datalog() { 

if (initialised == true) {

  if ((millis()-t13)>delaySD) { 

    t13 = millis(); 
    
      if (!SD.begin(chipSelect)) {
        sdok = false; 
      }
   
      else { 

        sdok = true; 

        dataFile = SD.open(namebuff, FILE_WRITE);

        if (dataFile) {  
          dataFile.println(mainSD);   
          dataFile.close(); 
        }
      } 
    }   
  }
}

//--------------------------------------//

void tlm() { 

  if (z == 0) { 
    
    if (millis()-t2>delayTLM) { 
      t2 = millis(); 
      sendtlm(); 
      
    } 
  } 
}

//--------------------------------------//

void sendtlm() { 

    Serial5.println(mainTLM);   
    //Serial.println(mainTLM);
  
}

void sdinit() { 
 
  if (!SD.begin(chipSelect)) { 
    
  }
   
  else { 

  dataFile = SD.open(namebuff, FILE_WRITE);

  if (dataFile) {  
      dataFile.println("Pwrontime, Mode, Gpsok, FailSafe, Stationary, Deployed, Vbatt, Looptime, Date, Time, Latitude, Longitude, AltGPS, CoG, Speed, Sat in use, HDOP, Posage, Fixtype, AltBARO, Vspeed, Magn, AltMERGED, BaroWeight, GPSWeight, Bearing, LatB, LonB, Offset, WindSpeed, WindDirection, MergeDir, Ch 0, Ch 1, Ch 2, Ch 3, Ch 4, Ch 5, Ch 6, ServoLeft, ServoRight, ServoDeploy");
      dataFile.close();

  }   
 } 
}

//--------------------------------------//
void epprom() {
  
  EEPROM.get(addr, count);
  sprintf(namebuff, "Data%0d.txt", count);
  EEPROM.write(addr, (count+1)); 
}

//--------------------------------------//

void eppclear() { 
  for ( unsigned int i = 0 ; i < EEPROM.length() ; i++ )
  EEPROM.write(i, 0); 
}

//--------------------------------------//

void autopilothome() { 
  
  int A = bearing ;
  int B = heading ;
  int C = cog ; 
  int D = 0; 

  float magnw = 1/(fwdspeed+1); 

  if (fwdspeed <= 0) { 
    magnw = 1;
  }
  
  float gpsw = (fwdspeed/2); 

  if (fwdspeed <= 0) {  
    gpsw = 0;
  }

  //----------------- AVERAGE Magn - GPS ---------------------//

    if (sqrt((B-C)*(B-C)) < 180) { 
      mergedir = ((B*magnw)+(C*gpsw))/(magnw+gpsw);
    }

    else { 
      mergedir = (int((((((B+180)%360)*magnw)+(((C+180)%360)*gpsw))/(magnw+gpsw))+180)%360);
    }

  D = mergedir; 
  //----------------- Offset calculation ---------------------//
   
    if (sqrt((A-D)*(A-D)) < 180) { 
      offst = (A-D);
    }

    else { 
    
      if ((A-D) < 0) {
        offst = 360 +(A-D);
      }

      else { 
       offst = (-360) + (A-D) ;
    }
   } 

   //----------------- Servo Command ---------------------//

   setservo(
   map(offst, -180,180, 1450, 1550),
   map(offst, -180,180, 1550, 1450),
   servo_deploy) ; 

}

//--------------------------------------//

void autopilotcap() { 
  
  int A = map(channels[5], 100, 1950, 0, 360) ;

  int D = mergedir; 
  //----------------- Offset calculation ---------------------//
   
    if (sqrt((A-D)*(A-D)) < 180) { 
      offst = (A-D);
    }

    else { 
    
      if ((A-D) < 0) {
        offst = 360 +(A-D);
      }

      else { 
       offst = (-360) + (A-D) ;
    }
   } 

   //----------------- Servo Command ---------------------//

   setservo(
   map(offst, -180,180, 1450, 1550),
   map(offst, -180,180, 1550, 1450),
   servo_deploy) ; 

}

//--------------------------------------------------- SERVO --------------------------------------------------- // 

void manualpilot() { 

  setservo(
  map(s_left.reading(channels[0]), 100, 1950, 1000, 2000),
  map(s_right.reading(channels[1]), 100, 1950, 1000, 2000),
  map(channels[6], 100, 1950, 1000, 2000)
  );

  if (failSafe == true) { 

    servo_left = 1500;
    servo_right = 1500; 
    
  }

  mixer(); 

}

//--------------------------------------//

void setservo(int a, int b, int c) {

  servo_left = a;
  servo_right = b;

  if (deployed == true) { 
  servo_deploy = 1500;
  }

  else { 
    
    if (failSafe == true) { 
      servo_deploy = 2000; 
    }
    
    else {
     servo_deploy = c;
    }
    
  }   
}

//--------------------------------------//

void updateservo(int a) { 

  if (millis()-t8>(1000/a)) {

    left.write(map(servo_left,1000,2000,0,180));
    right.write(map(servo_right,1000,2000,0,180));

    if (deployed == true) { 
    servo_deploy = 1500; 
    deploy.write(70);
    }

    else { 
    deploy.write(map(servo_deploy,1000,2000,40,140));
    }
  }
}

//--------------------------------------//

void mixer() { 
  
  float diff1 = sqrt(pow(1500-servo_left, 2)) ; 
  float diff2 = sqrt(pow(1500-servo_right, 2)) ; 

  float ail1 = 0; 
  float ail2 = 0;
  float elv1 = 0; 
  float elv2 = 0;
 

   if (1500-servo_left > 0) { 

   ail1 = 1500 - diff1 ;
   ail2 = 1500 + diff1 ;
    
   }
   else { 

   ail1 = 1500 + diff1 ;
   ail2 = 1500 - diff1 ;
    
   }

   if (1500-servo_right > 0) { 

   elv1 = 1500 + diff2 ;
   elv2 = 1500 + diff2 ;
    
   }
   else { 

   elv1 = 1500 - diff2 ;
   elv2 = 1500 - diff2 ;
    
   }
   
   //Serial.print(ail1); Serial.print(" ");
   //Serial.println(ail2);

   servo_left = (0.5 * ail1) + (0.5* elv1);
   servo_right = (0.5 * ail2) + (0.5* elv2); 

}

//--------------------------------------//

void getfailsafe() {
  
  if ((channels[3])>=1500 or (channels[3])==0) { failSafe = true; }
  else { failSafe = false; }
  
}

//--------------------------------------//

void getrc() {
  rx.read(&channels[0], &failSafe, &lostFrame);
}

//--------------------------------------------------- SENSOR --------------------------------------------------- // 

void geti2c() { 
  
  if ((micros()-t3)>10000) { 
    
    t3 = micros(); 
    
    compass.read();
    heading = 360 - ( ((compass.getAzimuth())+90) % 360 ) ;
    
    if ((bmp.readAltitude(1016))<(-500)) { delay(1500); }
    altitude = bmp.readAltitude(1016);

    if (initialised == true) {
      if ((millis()-t9)>delay5) { 

      t9 = millis(); 
      float da = ((altitude*offset_alt) - prev_alt);
      float vps = (da/(delay5/1000));
      vspeed = vs.reading(vps*100); 
      vspeed = (vspeed/100);
      prev_alt = (altitude*offset_alt); 
      
      }  
    } 
  }
}

//--------------------------------------//

void getvtimespeed() { 
  
   if ((millis()-t11) > 1000) { 
    
    t11 = millis(); 

    //Serial.println(vspeed_count); 
    
    if (vspeed < 0.2 and vspeed > -0.2) {
      vspeed_count = (vspeed_count + 1); 
    }
    else {
      vspeed_count = 0;  
    }
    
    if (vspeed_count >= 15) {
      stationary = true; 
    //Serial.println("Stationary!!!");
    }

    else {
      stationary = false; 
    }

    //Serial.println(stationary); 
  } 
}

//--------------------------------------//

void getcompass() { 

    compass.read();
    heading = 360 - ( ((compass.getAzimuth())+90) % 360 ) ;
     
} 

//--------------------------------------------------- GPS --------------------------------------------------- // 

void gpset(int a, int b, int c, int d, int e){
  
if (a == 9600) {
    Serial7.begin(9600); 
    delay(100); 
    byte packet1[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x92, 0xB5};
    sendPacket(packet1, sizeof(packet1));
  }
    
if (a == 57600) { 
    Serial7.begin(9600); 
    delay(100);
    byte packet2[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCE, 0xC9};
    sendPacket(packet2, sizeof(packet2));
    Serial7.end(); 
    Serial7.begin(57600); 
    delay(100);
  }

if (a == 115200) { 
    Serial7.begin(9600); 
    delay(100);
    byte packet3[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, 0x7E};
    sendPacket(packet3, sizeof(packet3)); 
    Serial7.end(); 
    Serial7.begin(115200);
    delay(100);
  }

if (b == 1) { 
    byte packet4[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39};
    sendPacket(packet4, sizeof(packet4));
  }

if (b == 5) { 
    byte packet5[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
    sendPacket(packet5, sizeof(packet5));
  }
    
if (b == 10) { 
    byte packet6[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
    sendPacket(packet6, sizeof(packet6));
  }


if (c == 0) { 
    byte packet7[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x3C};
    sendPacket(packet7, sizeof(packet7));
  }
    
if (c == 1) {
    byte packet8[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86, 0x4C};
    sendPacket(packet8, sizeof(packet8)); 
  }

if (d == 1) { 
  
    byte packet9[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x03, 0x35};
    byte packet10[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x05, 0x43};
    sendPacket(packet9, sizeof(packet9));
    sendPacket(packet10, sizeof(packet10));
  }

if (e == 1) {
    byte packet11[] = {0xB5, 0x62, 0x06, 0x1E, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x32, 0x00, 0x00, 0x99, 0x4C, 0x00, 0x00, 0x5B, 0x10};
    sendPacket(packet11, sizeof(packet11));
  } 
}

//--------------------------------------//

void sendPacket(byte *packet, byte len){
    for (byte i = 0; i < len; i++)
    {
        Serial7.write(packet[i]);
    }
}

//--------------------------------------//

void gpscheck() { 
  while (Serial7.available()) { 
    char c = Serial7.read();
    Serial.print(c); 
  }
}

//--------------------------------------//

void getgps() { 

  while (Serial7.available() > 0) {

    if (gps.encode(Serial7.read())) {
     
      newgps = true;
      t1 = millis(); 
      
      lata = gps.location.lat(); 
      lona = gps.location.lng(); 
      altgps = gps.altitude.meters(); 
      cog = gps.course.deg(); 
      twoDspeed = gps.speed.mps(); 
      dateday = gps.date.day();
      datemonth = gps.date.month();
      dateyear = gps.date.year();
      timehour = gps.time.hour();
      timeminute = gps.time.minute();
      timeseconde = gps.time.second();
      sats = gps.satellites.value();
      hdo = gps.hdop.value(); 
      fixtyped = atof(fixtype.value()); 
      posage = gps.location.age(); 

      bearing = TinyGPSPlus::courseTo(lata,lona,latb,lonb);

      if (posage>1100) { 
        posage = 999; 
      }    
  }
 }
}

// --------------------------------------------------- DATA COMPUTING --------------------------------------------------- //

void datetimecmpt() { 

  dtostrf(dateyear, 1, 0, date_year);
  dtostrf(datemonth, 1, 0, date_month);
  dtostrf(dateday, 1, 0, date_day);
  dtostrf(timehour, 1, 0, time_hour);
  dtostrf(timeminute, 1, 0, time_minute);
  dtostrf(timeseconde, 1, 0, time_seconde);
 
  snprintf(date_time, 80, "%s:%s:%s,%s:%s:%s", date_day, date_month, date_year, time_hour, time_minute, time_seconde);

}

//--------------------------------------//

void stat() { 
  
  if ((posage < 200) and (fixtyped == 3)) { 
    gpsok = true; 
  }

  else { 
    gpsok = false; 
  }
}

// --------------------------------------------------- INTERFACE --------------------------------------------------- //

void setled(int a, int b, int c, int d, int e) { 

  if (batt_critical == true) {

    a = 255; 
    b = 0; 
    c = 0; 
    d = 25;
    e = 50;
    
  }

  if (batt_low == true) { 
   
    a = 255; 
    b = 0; 
    c = 0; 
    d = 25;
    e = 200;
    
  }

  if ((millis()-lastled)>e) { 
    lastled = millis(); 
    colorWipe(strip.Color(b,a,c), 0);
    duration = d; 
    timeled = millis(); 
 }
}

//--------------------------------------//

void updateled() { 

  if ((millis()-timeled)>duration) { 
  colorWipe(strip.Color(0,0,0), 0);
  } 
}

//--------------------------------------//

void getcmd() { 
  
  while (Serial.available() > 0) {
    
    z = Serial.read(); 

    switch(z) {
      
    case '0': 
    flight_mode = 0; 
    break;

    case 'G': 
    data_time = 100; 
    flight_mode = 8; 
    break;

    case 'B': 
    data_time = 50; 
    flight_mode = 8; 
    break;

    case 'M': 
    data_time = 50; 
    flight_mode = 8; 
    break;

    case 'R': 
    data_time = 50; 
    flight_mode = 8; 
    break;

    case 'S': 
    data_time = 50; 
    flight_mode = 8; 
    break;

    case 'V': 
    data_time = 10; 
    flight_mode = 8; 
    break;

    case 'L': 
    flight_mode = 0; 
    break;

    case 'A': 
    data_time = 50; 
    flight_mode = 8; 
    break;
    
    
    }
  }

  while (Serial5.available() > 0) {
    
    z = Serial5.read(); 

    switch(z) {
      
    case '0': 
    flight_mode = 0; 
    break;

    case 'G': 
    data_time = 200; 
    flight_mode = 9; 
    break;

    case 'B': 
    data_time = 200; 
    flight_mode = 9; 
    break;

    case 'M': 
    data_time = 200; 
    flight_mode = 9; 
    break;

    case 'R': 
    data_time = 100; 
    flight_mode = 9; 
    break;

    case 'S': 
    data_time = 100; 
    flight_mode = 9; 
    break;

    case 'V': 
    data_time = 500; 
    flight_mode = 9; 
    break;

    case 'L': 
    flight_mode = 0; 
    break;

    case 'A': 
    data_time = 50; 
    flight_mode = 9; 
    break;
    
    
    }
  }
}

//--------------------------------------//

void checkmode() { 

  if (flight_mode != prev_mode) { 
    sendtlm(); 
    prev_mode = flight_mode; 
    Serial.println(flight_mode); 
  }
}

//--------------------------------------//

void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { 
    strip.setPixelColor(i, color);         
    strip.show();                          
  }
}

//--------------------------------------//
void looptime() { 

  loop_time = (t12-t10); 

  if (z == 'L') { 
  Serial.println(loop_time); 
  Serial5.println(loop_time); 
  }
}
//--------------------------------------//
void setcam(int a) { 
  
  switch(a) { 
    
    case 1 : 
    digitalWrite(cam, HIGH); 
    break; 

    case 0 : 
    digitalWrite(cam, LOW); 
    break; 
    
  }
}
