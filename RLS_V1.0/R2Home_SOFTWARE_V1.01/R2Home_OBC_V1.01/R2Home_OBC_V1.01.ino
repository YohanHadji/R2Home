#include <TinyGPS++.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include <SBUS.h>
#include <EasyBuzzer.h>
#include <Watchdog.h>
#include <Servo.h>
#include <movingAvg.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

// ----------------------------------- SETUP PANEL ----------------------------------- // 

#define i_want_to_fly   false // Simulated servo movement to test the servo movement :)) 
#define buzzer_turn     true  // Buzzer sounds as function of the turn command 
#define no_init         false // Skip init, for testing only purposes 
#define rc_mode         2     // 0 = only roll, 1 = pitch and roll mixed, 2 = pitch and roll separated
#define autopilot_mode  1     // Control linear or with "hands up"
#define drop            true // R2Home's version, drop or motorised
#define record_home     false // only record autopilot 
#define dep_alt         100   // m above ground  
#define vup             1.5   // m/s 
#define vdown           2     // m/s
#define gps_freq        5     // Hz
#define nav_waypoint    true  // Doing the waypoint sequence before reaching home? 
#define nav_home        false // Should it go home after the waypoint sequence? 

#define time_out 300

#define gps_port Serial7

#define bcritical 3.4
#define blow 3.5
#define no_batt 4.0 

#define servo_man_min 1100
#define servo_man_max 1900 

#define servo_auto_min 1100 
#define servo_auto_max 1900

#define servo_left_min 2000 
#define servo_left_start 1300
#define servo_left_max 1000

#define servo_right_min 2000 
#define servo_right_start 1300
#define servo_right_max 1000

#define gliding_timer 2500

#define waypoint_threshold 10 // Distance to waypoint before going to the next waypoint 

int dep_altitude = dep_alt; 
int cog_count = 2;  

// NAV PIDs // 
float NKp = 0.8; 
float NKd = 0.15; 

double long sim_cmd_time = 0; 
float sim_cmd = 0; 

// ----------------------------------- GLOBAL VARIABLES ----------------------------------- // 

// BARO // 
Adafruit_BMP280 bmp(&Wire);
int baro_adress = 0x00; 
float alt_baro = 0;
float prev_alt = 0; 
float vspeed = 0; 
float baro_set = 1000; 
float baro_count = 0; 
int vspeed_count = 0;
boolean new_baro = false;
float dt = 0; 
movingAvg al(5); 
movingAvg vs(5); 


// BATTERY // 
movingAvg voltage(50);  
float vpin = A17; 
float vbatt = 0; 
boolean batt_critical = false; 
boolean batt_low = false; 

// GPS // 
TinyGPSPlus gps;
TinyGPSCustom fix_type(gps, "GNGSA", 2);
movingAvg rs(2); 
unsigned char serial2bufferRead[1000];
float prev_cog = 0;
int gps_count = 0; 
int valid_count = 0; 
float prev_gps = 0; 
boolean new_gps = false; 
boolean cog_ok = 0; 
boolean new_cog = false; 


// LED // 
#define LED_PIN     3
#define LED_COUNT  1
#define BRIGHTNESS 15
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800); 
int duration = 0; 
int timeled = 0; 
int lastled = 0; 

// MISC // 
int buzzer = 2;
int cam = 23; 
int sw = 22; 
int crash_count = 0; 
int cells = 0; 
boolean armed = false;  

// NAV //   
movingAvg rc(2); 
movingAvg mult(10); 
float merged_alt = 0; 
float setPoint_Home = 0; 
float errHome = 0;
float raterror = 0; 
float last_errHome = 0; 
float baro_weight = 1;
float gps_weight = 1; 
float lat_B = 0; 
float lon_B = 0; 
float cmd = 0; 
boolean spiral = false; 
float cmdHome = 1500; 
float next_cog = 0; 
float ratecog = 0; 
float prev_cog_b = 0;
float cmd_mult = 0;  

struct gps_location { 
  double latitude = 0; 
  double longitude = 0; 
};

gps_location waypoint[17]; 
int waypoint_number = 0; 
int last_waypoint_number = 0; 

// RX // 
SBUS rx(Serial1);
movingAvg roll_steer(25);
movingAvg pitch_steer(25);
uint16_t channels[16];
bool failSafe;
bool lostFrame;

// SD CARD // 
File dataFile; 
File configFile; 
const int chipSelect = BUILTIN_SDCARD;
char namebuff[20]; 
unsigned int addr = 0;
String filename; 
int datatest = 0; 
int count = 0; 

// SERVO /  
movingAvg st(5); 
Servo steer;
Servo deploy; 
Servo left; 
Servo right; 
Servo esc; 

float roll_man = 1500; 
float pitch_man = 1500; 
float steer_auto = 1500; 
 
float servo_deploy = 2000;
float servo_esc = 1000; 

float servo_left = 1500; 
float servo_right = 1500; 

// STATUS // 
int flight_mode = 0; 
int prev_mode = 0; 
bool deployed = false; 
bool baro_stab = false; 
bool gps_stab = false; 
bool initialised = false;
bool wing_opened = false;  

bool sd_ok = false; 
bool gps_ok = false; 

int packet_count = 0; 
int reboot_state = 0; 

// STRING //
char sdnamebuff[20]; 
char mainSD[240];
char mainTLM[250];   

int time_number = 0; 
boolean flight_rebooted = false; 

// TIMERS // 
unsigned long baroA = 0; 
unsigned long baroB = 0; 
unsigned long tstab = 0; 
unsigned long tlm = 0; 
unsigned long sd = 0; 
unsigned long baro_blk = 0; 
unsigned long init_time = 0; 
unsigned long sat_buzz = 0; 
unsigned long tparallax = 0; 
unsigned long tpwm = 0; 
unsigned long batt_buzz = 0; 
unsigned long long tloop = 0; 
unsigned long long tgps = 0; 
unsigned long long tup = 0; 
unsigned long long tdown = 0; 
unsigned long t_turn = 0; 
unsigned long tlooptime = 0; 
unsigned long mes = 0; 

unsigned long long reboot_time = 0; 

int loop_time = 999;
int loop_time_min_loc = 999; 
int loop_time_min_glob = 999; 
int loop_time_max_loc = 999; 
int loop_time_max_glob = 999; 
double loop_time_mean = 999; 
int loop_time_count = 0; 


int delaySD = 100;    // Datalog 
int delayTLM = 1000;   // Tlm 


// WATCHDOG // 
Watchdog watchdog;


void setup() {

  //eppclear();  
  
  pinMode(A17, INPUT); 
  pinMode(buzzer, OUTPUT);
  pinMode(cam, OUTPUT);
  pinMode(sw, INPUT); 
  digitalWrite(cam, LOW); 
  
  Serial.begin(115200);
  Serial5.begin(57600); 

  gpset(57600, gps_freq, 2, 1, 0); // baud, Hz, mode, nmea, cog filter (0 = Off, 1 = On)
  
  rx.begin();
   
  EasyBuzzer.setPin(buzzer);

  voltage.begin();
  vs.begin(); 
  al.begin(); 
  roll_steer.begin();
  pitch_steer.begin();
  rs.begin(); 
  rc.begin();
  mult.begin(); 

  left.attach(6, 1000, 2000);  
  right.attach(7, 1000, 2000); 
  
  if (drop == true) { deploy.attach(8, 1000, 2000); } 
  else              { esc.attach(8, 1000, 2000);    }

  getconfig(); 
  bmp.begin(baro_adress); 
  
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
  Adafruit_BMP280::SAMPLING_X1,   
  Adafruit_BMP280::SAMPLING_X2,    
  Adafruit_BMP280::FILTER_X16,      
  Adafruit_BMP280::STANDBY_MS_1); 
  
  strip.begin();           
  strip.show();            
  strip.setBrightness(BRIGHTNESS);

  tone(buzzer, 523); 
  delay(200); 
  tone(buzzer, 582); 
  delay(200); 
  tone(buzzer, 762); 
  delay(300); 
  noTone(buzzer);

  EEPROM.get(0, reboot_state);

  if (reboot_state == 1) { 
    EEPROM.get(10, dep_altitude);
    EEPROM.get(30, lat_B);
    EEPROM.get(50, lon_B);
    EEPROM.get(70, baro_set);
    EEPROM.get(90, time_number);
    EEPROM.get(120, reboot_time);

    flight_rebooted = true; 
  }

 watchdog.enable(Watchdog::TIMEOUT_1S);
 
}

void loop() {
  
  tloop = micros(); 

  sim(); 

  getdata();
  datacmpt(); 
  
  flight_state(); 
  
  applycmd(); 
  updatecmd(50);
  
  userinter(); 
  
  loop_time = micros()-tloop;


  if (loop_time_count >= 100) {
    loop_time_min_glob = loop_time_min_loc;
    loop_time_max_glob = loop_time_max_loc;
    loop_time_min_loc = 999; 
    loop_time_max_loc = 999; 
    loop_time_mean = (micros()-tlooptime)/100;
    tlooptime = micros(); 
    loop_time_count = 0; 
  }

  else if (loop_time_count < 100) {
    if (loop_time<loop_time_min_loc) {loop_time_min_loc = loop_time; }
    if (loop_time>loop_time_max_loc) {loop_time_max_loc = loop_time; }
    loop_time_count++; 
  }

 
  if (loop_time<250000) {watchdog.reset(); crash_count = 0; }
  if (loop_time>250000) { 
    crash_count = (crash_count + 1); 
    if (crash_count<=15) { watchdog.reset(); }
    else { 
    reboot_state = 1; 
    EEPROM.put(0, reboot_state);
    EEPROM.put(120, millis()); 
    }
  }

}


void getdata() { 

  // -------------------------- Get GPS -------------------------- // 

 while (Serial7.available()) { gps.encode(Serial7.read()); }


  // -------------------------- Get BARO & COMPASS -------------------------- //

  if ((millis()-baroA)>=10) { 
    baroA = millis();
    unsigned waitd = millis(); 
    alt_baro = al.reading(bmp.readAltitude(baro_set)*100);
    if (((millis() - waitd) > 100)) { EasyBuzzer.singleBeep(3000,50); bmp.begin(baro_adress); bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X1, Adafruit_BMP280::SAMPLING_X2,  Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_1); } 
    alt_baro = (alt_baro/100);
    baro_count = (baro_count + 1);;
     
    if (baro_count >= 10) { 
      baro_count = 0; 
      new_baro = true; 
      dt = baroA-baroB;
      baroB = millis(); 
    } 
  }

  // -------------------------- Get RC -------------------------- //

  rx.read(&channels[0], &failSafe, &lostFrame);
  
  if ((channels[3])>=1500 or (channels[3])==0) { failSafe = true; }
  else { failSafe = false; } 

  roll_man  = map(roll_steer.reading(channels[0]), 50, 1950, 1000, 2000);
  pitch_man = map(pitch_steer.reading(channels[1]), 50, 1950, 1000, 2000);
  roll_man  = constrain(roll_man, servo_man_min, servo_man_max); 
  pitch_man = constrain(pitch_man, servo_man_min, servo_man_max); 

// -------------------------- Get Vbatt -------------------------- //

  vbatt = analogRead(A17); 
  vbatt = (voltage.reading(vbatt)*0.02579582875);


}

// ----------------------------------------------------- Data CMPT ----------------------------------------------------- //

void datacmpt() {

// -------------------------- NAV -------------------------- //

// -------------------- GPS Healt -------------------- //
  
  if (((gps.location.age()) < 999) and (atof(fix_type.value()) == 3)) { gps_ok = true; }
  else { gps_ok = false; }


// -------------------------- STUFF DEPENDING ON GPS COURSE OVER GROUND -------------------------- //

// -------------------- Autopilot -------------------- //

  if (gps.course.isUpdated()) {  
    new_cog = true;                                                 
    if (cog_valid(cog_count) == true) {
      cog_ok = true;                                                    
      setPoint_Home = TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),lat_B,lon_B);  
      errHome = getangle(gps.course.deg(), setPoint_Home);                                    
    }
    else { cog_ok = false; last_errHome = errHome; }                                                                 
    prev_cog = gps.course.deg();                                                        
  }

   
  if ((gps_ok == true) and (cog_ok == true)) { 
   
      if (new_cog == true) {
        
        raterror = getangle((last_errHome+180),(errHome+180))*gps_freq ; raterror = rs.reading(raterror); 
        ratecog  = getangle(prev_cog_b,gps.course.deg())*gps_freq;       ratecog  = rc.reading(ratecog);
        
        float PIDsum = ((NKp*errHome)+(NKd*raterror)); 
        
        //cmd_mult = (0.09999996 + (1 - 0.09999996)/(1 + pow(((abs(ratecog))/49.54231),24.26521))); cmd_mult = mult.reading(cmd_mult*1000); cmd_mult = (cmd_mult/1000); 

        cmd_mult = 1;  
        cmdHome  = PIDsum*cmd_mult ;
        
        //if (TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),lat_B,lon_B)<5) { cmdHome = 180; } 
        if (vspeed<-5) { spiral = true; } 
        if (vspeed>-3) { spiral = false; } 
        if (spiral == true) { cmdHome = 0; }
    
        new_cog = false;
        last_errHome = errHome; 
        prev_cog_b = gps.course.deg(); 

        steer_auto = map(cmdHome, -180, +180, 1000, 2000); 
        steer_auto = constrain(steer_auto, servo_auto_min, servo_auto_max);  
     
    }    
  }
  
 else { steer_auto = 1500; }

 if (i_want_to_fly == true) { 
  flight_mode = 5; 
  steer_auto = map(sim_cmd, -180, +180, 1000, 2000); 
 }
  
   
// -------------------- Vertical Speed -------------------- //

 if (new_baro == true) { 
  
  new_baro = false; 
  
  if ((initialised == false) and (reboot_state != 1)) { baro_set = (baro_set + ((0-alt_baro)/100)); }
  if (millis()<=5000) { alt_baro = 0; } 
  
  float da = (alt_baro - prev_alt);
  
  if (abs(da) < 50) {
  float vps = (da/(dt/1000));
  vspeed = vs.reading(vps*100); 
  vspeed = (vspeed/100);
  } 
  
  prev_alt = alt_baro; 

}

// -------------------- Alt Fusion -------------------- //

    baro_weight = (1+(abs(vspeed)/10));
    
    float hdo = gps.hdop.value(); 
    float gpsw = ((50/hdo)+(gps.altitude.meters()/10000)); 
    gps_weight = (gpsw*((abs(alt_baro-gps.altitude.meters())/10)));
    
    merged_alt = ((alt_baro*baro_weight)+(gps.altitude.meters()*gps_weight))/(baro_weight+gps_weight); 


// -------------------- Stationary -------------------- //

  if ((millis()-tstab) >= 1000) { 
    tstab = millis(); 
    
    if (vspeed < 0.2 and vspeed > -0.2) { vspeed_count = (vspeed_count + 1); }
    else { vspeed_count = 0; }

    if (abs(prev_gps-gps.altitude.meters())<1 and (gps.altitude.meters() != 0)) { gps_count = (gps_count + 1); }
    else { gps_count = 0; } 

    if (gps_count >= 10) { gps_stab = true; } 
    else if(millis()<(time_out*1000)) { gps_stab = false; } 
    else { gps_stab = true; } 
    
    
    if (vspeed_count >= 10) { baro_stab = true; }
    else { baro_stab = false; }

    if (gps_count >= 10) { gps_stab = true; } 
    else { gps_stab = false; } 
    
    prev_gps = gps.altitude.meters();  


  }   

// -------------------------- String -------------------------- //

  char lat_A_text[30];
  char lon_A_text[30];
  char alt_gps_text[30];
  char cog_text[30]; 
  char speed_text[30];
  char sat_text[30];
  char fix_type_text[10];
  char hdop_text[10];
  char pos_age_text[10]; 
  char count_text[10]; 
  char time_text[10];

  char alt_baro_text[30];
  char vspeed_text[30];

  char merged_alt_text[30];
  char baro_weight_text[30];
  char gps_weight_text[30]; 
  char setPoint_Home_text[30]; 
  char lat_B_text[30];
  char lon_B_text[30];
  char errHome_text[30]; 
  char raterror_text[30]; 

  char a_text[10]; 
  char b_text[10]; 
  char c_text[10]; 
  char d_text[10]; 
  char e_text[10]; 
  char f_text[10]; 
  char g_text[10]; 
  
  char deploy_text[10]; 
  char left_text[10]; 
  char right_text[10]; 
  char esc_text[10]; 
  
  char gps_ok_text[10];
  char cog_ok_text[10];    
  char failSafe_text[10]; 
  char baro_stab_text[10]; 
  char gps_stab_text[10]; 
  char deployed_text[10]; 
  char flight_mode_text[10]; 
  char vbatt_text[10]; 
  char loopmax_text[10];
  char loopmin_text[10];  
  char loopmean_text[10];  
  char packet_count_text[10]; 
  char initialised_text[10]; 
  char wing_opened_text[10]; 
  char next_cog_text[10]; 
  char cmd_mult_text[10]; 
  
  char mintext[120];
  char gps_text[120]; 
  char baro_text[120];
  char rc_text[120]; 
  char servo_text[120]; 
  char status_text[120]; 
  char alt_text[120]; 
  char nav_text[120]; 
  
  char date_time[80];
  
  char date_year[10]; 
  char date_month[10]; 
  char date_day[10]; 
  char time_hour[10]; 
  char time_minute[10];
  char time_seconde[10];
  
  dtostrf(gps.date.year(), 2, 0, date_year);
  dtostrf(gps.date.month(), 1, 0, date_month);
  dtostrf(gps.date.day(), 1, 0, date_day);

  dtostrf(gps.time.hour(), 1, 0, time_hour);
  dtostrf(gps.time.minute(), 1, 0, time_minute);
  dtostrf(gps.time.second(), 1, 0, time_seconde);
   
 
  snprintf(date_time, 100, "%s:%s:%s,%s:%s:%s", date_year, date_month, date_day, time_hour, time_minute, time_seconde);

  if (flight_rebooted == false) { time_number = ((gps.date.day()*1000000) + (gps.time.hour()*10000) + (gps.time.minute()*100) + gps.time.second()); } 

  dtostrf(gps.location.lat(), 10, 10, lat_A_text);
  dtostrf(gps.location.lng(), 10, 10, lon_A_text);
  dtostrf(gps.altitude.meters(), 1, 1, alt_gps_text);
  dtostrf(gps.course.deg(), 1, 0, cog_text);
  dtostrf(gps.speed.mps(), 1, 1, speed_text);
  dtostrf(gps.satellites.value(), 1, 0, sat_text);
  dtostrf(atof(fix_type.value()), 1, 0, fix_type_text);
  dtostrf(gps.hdop.value(), 1, 0, hdop_text); 
  
  if (gps.location.age()>10000) { dtostrf(9999, 1, 0, pos_age_text); } 
  else                         { dtostrf(gps.location.age(), 1, 0, pos_age_text); } 

  snprintf(gps_text, 120, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s", date_time, lat_A_text, lon_A_text, alt_gps_text, cog_text, speed_text, sat_text, hdop_text, pos_age_text, fix_type_text);

  dtostrf(alt_baro, 1, 3, alt_baro_text);
  dtostrf(vspeed, 1, 2, vspeed_text);
  
  snprintf(baro_text, 120, "%s,%s", alt_baro_text, vspeed_text);  
  
  dtostrf(merged_alt, 2, 3, merged_alt_text);  
  dtostrf(gps_weight, 2, 2, gps_weight_text);
  dtostrf(baro_weight, 2, 3, baro_weight_text);
  
  dtostrf(setPoint_Home, 2, 2, setPoint_Home_text);
  dtostrf(errHome, 1, 2, errHome_text);
  dtostrf(raterror, 1, 0, raterror_text); 
  dtostrf(cmd_mult, 1, 3, cmd_mult_text); 
  
  dtostrf(lat_B, 1, 5, lat_B_text);
  dtostrf(lon_B, 1, 5, lon_B_text);

  dtostrf(next_cog, 1, 0, next_cog_text); 
      
  snprintf(nav_text, 140, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s", merged_alt_text, baro_weight_text, gps_weight_text, setPoint_Home_text, errHome_text, raterror_text, next_cog_text, cmd_mult_text, lat_B_text, lon_B_text);  

  dtostrf(channels[0], 2, 0, a_text);
  dtostrf(channels[1], 2, 0, b_text);
  dtostrf(channels[2], 2, 0, c_text);
  dtostrf(channels[3], 2, 0, d_text);
  dtostrf(channels[4], 2, 0, e_text);
  dtostrf(channels[5], 2, 0, f_text);
  dtostrf(channels[6], 2, 0, g_text);

  snprintf(rc_text, 120, "%s,%s,%s,%s,%s,%s,%s", a_text, b_text, c_text, d_text, e_text, f_text, g_text);

  if (drop == true) { 
    dtostrf(servo_left, 2, 0, left_text);
    dtostrf(servo_right, 2, 0, right_text);
    dtostrf(servo_deploy, 2, 0, deploy_text);
    snprintf(servo_text, 40, "%s,%s,%s", left_text, right_text, deploy_text); 
  }

  else { 
    dtostrf(servo_left, 2, 0, left_text);
    dtostrf(servo_right, 2, 0, right_text);
    dtostrf(servo_esc, 2, 0, esc_text);
    snprintf(servo_text, 40, "%s,%s,%s", left_text, right_text, esc_text);   
  }

  unsigned long long actual_time = (millis()+reboot_time); 
  dtostrf(actual_time, 1, 0, time_text);
  dtostrf(flight_mode, 1, 0, flight_mode_text);
  dtostrf(gps_ok, 1, 0, gps_ok_text);
  dtostrf(cog_ok, 1, 0, cog_ok_text);
  dtostrf(failSafe, 1, 0, failSafe_text);
  dtostrf(gps_stab, 1, 0, gps_stab_text);
  dtostrf(baro_stab, 1, 0, baro_stab_text);
  dtostrf(deployed, 1, 0, deployed_text);
  dtostrf(wing_opened, 1, 0, wing_opened_text);
  dtostrf(vbatt, 1, 3, vbatt_text);
  dtostrf(loop_time_min_glob, 1, 0, loopmin_text);
  dtostrf(loop_time_max_glob, 1, 0, loopmax_text);
  dtostrf(loop_time_mean, 1, 0, loopmean_text);
  dtostrf(packet_count, 1, 0, packet_count_text);
  dtostrf(initialised, 1, 0, initialised_text);
  
  snprintf(status_text, 100, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s", time_text, packet_count_text, flight_mode_text, gps_ok_text, cog_ok_text, failSafe_text, gps_stab_text, baro_stab_text, deployed_text, wing_opened_text, initialised_text, vbatt_text, loopmin_text, loopmax_text, loopmean_text);  

  snprintf(mainSD, 340, "%s,%s,%s,%s,%s,%s", status_text, gps_text, baro_text, nav_text, rc_text, servo_text);

  snprintf(mainTLM, 250, "/*%s,%s,%s,%s*/", status_text, gps_text, baro_text, servo_text);

// -------------------------- TLM -------------------------- //

  switch(flight_mode) { 
      
    case 0: 
    delaySD = 200;
    delayTLM = 5000;  
    break;

    case 1: 
    case 2: 
    case 3: 
    case 4: 
    case 5:
    case 6: 
    case 8:
    case 9:  
    delaySD = 50;
    delayTLM = 1000;  
    break;
      
  }
    
  if (millis()-tlm>=delayTLM) { 
    tlm = millis(); 
    packet_count = (packet_count +1); 
    Serial5.println(mainTLM);   
    Serial.println(mainTLM); 
    
    if (sd_ok == true) {
        dataFile = SD.open(namebuff, FILE_WRITE);
        if (dataFile) { dataFile.println(mainSD); dataFile.close(); }
        else { sd_ok = false; }
    }
      
  } 
 
// -------------------------- SD DATALOG -------------------------- //

if (record_home == false) { 

  if (initialised == true) {

    if ((millis()-sd)>=delaySD) { 
      sd = millis(); 
     
      if (sd_ok == true) {
        dataFile = SD.open(namebuff, FILE_WRITE);
        if (dataFile) { dataFile.println(mainSD); dataFile.close(); }
        else { sd_ok = false; }
      }
   
    
   } 
  }
 }
 
else {

   if ((flight_mode == 9) or (flight_mode == 10)) {

    if ((millis()-sd)>=delaySD) { 
      sd = millis(); 
 
    
      if (sd_ok == true) { 
        dataFile = SD.open(namebuff, FILE_WRITE);
        if (dataFile) { dataFile.println(mainSD); dataFile.close(); }
        else { sd_ok = false; }
    }   
   }
  }
 }
}

// ----------------------------------------------------- State machine ----------------------------------------------------- //


void flight_state() {

  if (flight_mode != prev_mode) { 

    packet_count = (packet_count +1); 
    Serial5.println(mainTLM);   
    Serial.println(mainTLM); 
    
    if (sd_ok == true) {
      dataFile = SD.open(namebuff, FILE_WRITE);
      if (dataFile) { dataFile.println(mainSD); dataFile.close(); }
    }
     
   prev_mode = flight_mode;  
 }
  
  switch(flight_mode) { 
    
    case 0: 
      flight_init(); 
      setled(255, 0, 0, 25, 2000);
    break;

    case 1: 
      ready_steady(); 
      setled(0, 255, 0, 25, 500);
    break;

    case 2: 
      flight_ascent(); 
      setled(0, 0, 255, 25, 2000); 
    break;

    case 3: 
      flight_descent();
      setled(255, 128, 0, 25, 1000);  
    break;

    case 4: 
      flight_gliding(); 
    break;

    case 5: 
      flight_gliding_auto(); 
      setled(255, 255, 0, 25, 1000); 
    break;

    case 6: 
      flight_gliding_manual(); 
      setled(0, 255, 255, 25, 1000);
    break;

    case 7: 
      landed(); 
      setled(128, 0, 255, 25, 1000); 
    break;

    case 8: 
      motorised_man(); 
      setled(0, 0, 255, 25, 500); 
    break;

    case 9: 
      motorised_auto(); 
      setled(0, 255, 255, 25, 500);
    break; 

    case 10: 
      motorised_failSafe(); 
      setled(0, 255, 255, 25, 500);
    break; 
    
  }
}
//------------------- 0 -------------------//

void flight_init() { 

 if (reboot_state !=1) { 
  
  if (((gps.satellites.value() >= 6) and (gps_ok == true) and (gps_stab == true) and (millis()>5000)) or (no_init == true)) {
   
    EasyBuzzer.beep(3000,100,50,10,500,1);

    if ((nav_waypoint == true) and (nav_home == true)) {
      waypoint[last_waypoint_number].latitude = gps.location.lat();
      waypoint[last_waypoint_number].longitude = gps.location.lng();
    }

    else if ((nav_waypoint == false) and (nav_home == true)) {
      lat_B = gps.location.lat();
      lon_B = gps.location.lng();
    }

    else if ((nav_waypoint == true) and (nav_home == false)) {
      lat_B = waypoint[waypoint_number].latitude; 
      lon_B = waypoint[waypoint_number].longitude; 
      waypoint_number++; 
    }

    initialised = true; 

    EEPROM.put(10, dep_altitude);
    EEPROM.put(30, lat_B);
    EEPROM.put(50, lon_B);

    while (abs(alt_baro-gps.altitude.meters())>0.01) {
      if ((micros()-baro_blk)>10) { 
        baro_blk = millis();
        alt_baro = (bmp.readAltitude(baro_set));
        baro_set = (baro_set + ((gps.altitude.meters()-alt_baro)/8));
        prev_alt = alt_baro; 
        watchdog.reset();
      }
    }

    EEPROM.put(70, baro_set);

    vs.reset(); 
    al.reset(); 
  
    dep_altitude = (dep_altitude+gps.altitude.meters());

    strip.setBrightness(50);
    setcam(1); 
  
    if (record_home == false) { newfile(); }   

    if (drop == true) { flight_mode = 1;}
    else { flight_mode = 8; }

    init_time = millis(); 
         
  }
 }

 else { 
 
  if ((gps.satellites.value() >= 6) and (gps_ok == true) and (millis()>5000)) {
   
    EasyBuzzer.beep(3000,100,50,10,500,1);
    initialised = true; 

    vs.reset(); 
    al.reset(); 
  
    strip.setBrightness(50);
    setcam(1); 
      
    if (drop == true) { flight_mode = 1;}
    else { flight_mode = 8; }
    
    init_time = millis();

    dtostrf(time_number, 1, 0, sdnamebuff); 
    sprintf(namebuff, "%s.txt", sdnamebuff);

    reboot_state = 0; 
    EEPROM.put(0, 0); 
      
  }
 }
  
  if (millis()-sat_buzz>5000) { 
    sat_buzz = millis(); 
    EasyBuzzer.beep(2000,50,25,gps.satellites.value(),100,1); 
  } 
   
}

//------------------- 1 -------------------//

void ready_steady() { 

 if (millis()-init_time>=1000) { 
   
  if (vspeed>vup)    { flight_mode = 2; EasyBuzzer.beep(1000,100,50,2,500,1); strip.setBrightness(75);  }
  if (vspeed<vdown)  { flight_mode = 3; EasyBuzzer.beep(3000,100,50,3,500,1); strip.setBrightness(100); } 
  if ((atof(fix_type.value()) < 2) and (no_init == false))  { flight_mode = 0; EasyBuzzer.beep(700,100,50,3,500,1);  strip.setBrightness(50);  }
 
 }
} 

//------------------- 2 -------------------//

void flight_ascent() { if (vspeed<0.5) {flight_mode = 1; strip.setBrightness(50);}  }

//------------------- 3 -------------------//

void flight_descent() { 
  
  if (vspeed>-0.5) {flight_mode = 1; strip.setBrightness(50);} 
  if (merged_alt < dep_altitude) { flight_mode = 4; EasyBuzzer.beep(3000,100,50,5,500,1); strip.setBrightness(255); deployed = true; init_time = millis(); }
     
 }   

//------------------- 4 -------------------//

void flight_gliding() {

  if (((millis()-init_time) >= gliding_timer) or (vspeed > (vdown+1))) { 
  wing_opened = true; 
  last_errHome = errHome; 
  if (failSafe == true) {flight_mode = 5; strip.setBrightness(100);} 
  else                  {flight_mode = 6; strip.setBrightness(100);}
  } 
}

//------------------- 5 -------------------//

void flight_gliding_auto() { 
  
  if (failSafe == false) {flight_mode = 6;}  
  if (vspeed < -6) { spiral = true; } 
  if (vspeed > -5) { spiral = false; } 

  if (i_want_to_fly == true) { spiral = false; flight_mode = 5; } 
  
  navigation(); 
}

//------------------- 6 -------------------//

void flight_gliding_manual() { 
  if (failSafe == true) {flight_mode = 5;}  
}

//------------------- 7 -------------------//

void landed() { if ((baro_stab == false) or (gps_stab == false)) {flight_mode = 1; strip.setBrightness(50);} }

//------------------- 8 -------------------//

void motorised_man() {

  if (channels[6] > 1000) { flight_mode = 9; if (record_home == true) {newfile();} } 
  if (failSafe == true) { flight_mode = 10; if (record_home == true) {newfile();} }
}

//------------------- 9 -------------------//

void motorised_auto() { 
  
  navigation(); 
  
  if (channels[6] < 1000) { flight_mode = 8; }
}

//------------------- 10 -------------------//

void motorised_failSafe() { 
  
  navigation(); 
  
  if (failSafe == false) { flight_mode = 8; }
}
// ----------------------------------------------------- Apply command ----------------------------------------------------- //

void applycmd()  { 

// -------------------------- Steering Servo -------------------------- //

  switch(flight_mode) { 
    
    case 0: 
    case 1: 
    case 6: 
      if (failSafe == false) { 
        switch (rc_mode) {
          
          case 0: 
          servo_left = roll_man; 
          servo_right = 3000-roll_man; 
          break; 

          case 1:
          servo_left = roll_man; 
          servo_right = 3000-roll_man; 
   
          servo_left  = servo_left+(pitch_man-1500); 
          servo_right = servo_right+(pitch_man-1500); 
          
          servo_left = constrain(servo_left, servo_man_min, servo_man_max);
          servo_right = constrain(servo_left, servo_man_min, servo_man_max);
          break; 

          case 2: 
          servo_left = roll_man; 
          servo_right = pitch_man;
          break; 
        }
      }
      else { servo_left = 1500; servo_right = 1500; }  
    break; 

    case 8: 
      switch (rc_mode) {
          case 0: 
          servo_left = roll_man; 
          servo_right = 3000-roll_man; 
          break; 

          case 1:
          servo_left = roll_man; 
          servo_right = 3000-roll_man; 
   
          servo_left  = servo_left+(pitch_man-1500); 
          servo_right = servo_right+(pitch_man-1500); 
          
          servo_left = constrain(servo_left, servo_man_min, servo_man_max);
          servo_right = constrain(servo_left, servo_man_min, servo_man_max);
          break; 

          case 2: 
          servo_left = roll_man; 
          servo_right = pitch_man;
          break; 
        }
    break;

    case 9:
    case 10: 
    case 5: 
    
      if (autopilot_mode == 0) { 
        servo_left = steer_auto; 
        servo_right = 3000-steer_auto; 
      }
      
      else {
        
        if (steer_auto < 1450) {
         servo_left = map(steer_auto, 1000, 1500, 2000, servo_left_start);
         servo_right = servo_right_max;
        }
           
        else if (steer_auto > 1550) { 
         servo_right = map(steer_auto, 2000, 1500, 2000, servo_right_start);
         servo_left = servo_left_max; 
        }
        
        else if ((steer_auto>1450) and (steer_auto<1550)) { 
          servo_left = servo_left_max; 
          servo_right = servo_right_max; 
        }
        
      } 
    break;
  
    case 2: 
    case 3: 
    case 4: 
      servo_left = 1500; 
      servo_right = 1500; 
    break; 
    
  }
// -------------------------- Deployment Servo and ESC -------------------------- //

  switch(flight_mode) { 
    
  
    case 0: 
    case 1:
    case 2:
    case 3:
    case 4:
    case 5: 
    case 6:
    case 7: 
     if (deployed == true) { servo_deploy = 1500; }
     else {
       if (failSafe == true) { servo_deploy = 2000; } 
       else { servo_deploy = map(channels[6], 50, 1950, 1000, 2000);}
      }
   break; 

   case 8:
     if ((channels[4]>1000) and (channels[2]<400)) { armed = true;  }
     if (channels[4]<1000) { armed = false; }
     
     if (armed == true) { servo_esc = map(channels[2], 50, 1950, 1000, 2000); }
     else { servo_esc = 900; }
     
     if ((channels[4]>1000) and (channels[2]>400) and (armed == false)) { EasyBuzzer.singleBeep(3000,25);    }
  
   break; 

   case 9:
   case 10:  
    servo_esc = 1000; 
   break; 

  }  
}

// ----------------------------------------------------- Update command ----------------------------------------------------- //

void updatecmd(float a) { 

if (drop == true) {

    if ((millis()-tpwm)>=(1000/a)) { 
    
    tpwm = millis(); 
    left.writeMicroseconds(servo_left);
    right.writeMicroseconds(servo_right); 
    deploy.writeMicroseconds(servo_deploy);
    
  }
 }

else {
  
  if ((millis()-tpwm)>=(1000/a)) { 
    
    tpwm = millis(); 
    left.writeMicroseconds(servo_left);
    right.writeMicroseconds(servo_right); 
    esc.writeMicroseconds(servo_esc);
    
  }
 } 
}

// ----------------------------------------------------- User Interface ----------------------------------------------------- //

void userinter() { 

// -------------------------- Buzzer -------------------------- //

  EasyBuzzer.update();


  if (buzzer_turn == true and flight_mode == 5) { 
    
    int tone_turn = map(steer_auto, 1000, 2000, 1000, 3000); 
    double t_down    = abs(map(steer_auto, 1000, 2000, -9, 9))+1; 
    t_down = (1/t_down)*500; 
   

    if ((millis()-t_turn)>t_down) { 
      t_turn = millis(); 
      EasyBuzzer.singleBeep(tone_turn,10);
    }
    
  }
  
// -------------------------- Voltage monitoring -------------------------- //

  cells = int(vbatt/3.65); 

  if ((vbatt < (cells*bcritical)) and (vbatt > no_batt)) { 
    batt_critical = true; 
    if (millis()-batt_buzz>=100) { 
      batt_buzz = millis(); 
      EasyBuzzer.singleBeep(3000,25);
    }
  }
  
  else { batt_critical = false; }

  if ((vbatt < (cells*blow)) and (vbatt > (cells*bcritical))) {  
    batt_low = true; 
    if (millis()-batt_buzz>=200) { 
      batt_buzz = millis(); 
      EasyBuzzer.singleBeep(3000,50);
    }
  }

  else { batt_low = false; }
  

// -------------------------- RGB LED -------------------------- //

  updateled(); 

} 

// ----------------------------------------------------- Misc Functions ----------------------------------------------------- //

void setled(int a, int b, int c, int d, int e) { 
  
  if (batt_critical == true) { a = 255; b = 0; c = 0; d = 25; e = 50; }
  if (batt_low == true) { a = 255; b = 0; c = 0; d = 25; e = 200; }

  if ((millis()-lastled)>=e) { 
    lastled = millis(); 
    colorWipe(strip.Color(b,a,c), 0);
    duration = d; 
    timeled = millis(); 
 }
}

void updateled() { 
  if ((millis()-timeled)>=duration) { 
    colorWipe(strip.Color(0,0,0), 0);
 }  
}

void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { 
    strip.setPixelColor(i, color);         
    strip.show();                          
  }
}

boolean cog_valid(int a) { 
  if (abs(gps.course.deg()-prev_cog)<0.1) { valid_count++; }
  else { valid_count = 0; } 
  if (valid_count >= a) { return false; }
  else { return true; } 
}

float getangle(float a, float b) { 
  float angle = 0; 
  if (abs(a-b) < 180) { angle = (b-a);}
  else { 
    if ((a-b) < 0) { angle = (-360) +(b-a);}
    else { angle = (360) + (b-a);}
  }
  return angle;  
}

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

void dateTime(uint16_t* date, uint16_t* time){
  
  *date = FAT_DATE(gps.date.year(), gps.date.month(), gps.date.day());
  *time = FAT_TIME(gps.time.hour(), gps.time.minute(), gps.time.second());
}

void gpset(int a, int b, int c, int d, int e){
  
if (a == 9600) {
    gps_port.begin(9600); 
    delay(100); 
    byte packet1[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x92, 0xB5};
    sendPacket(packet1, sizeof(packet1));
  }
    
if (a == 57600) { 
    gps_port.begin(9600); 
    delay(100);
    byte packet2[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCE, 0xC9};
    sendPacket(packet2, sizeof(packet2));
    gps_port.end(); 
    gps_port.begin(57600); 
    delay(100);
  }

if (a == 115200) { 
    gps_port.begin(9600); 
    delay(100);
    byte packet3[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, 0x7E};
    sendPacket(packet3, sizeof(packet3)); 
    gps_port.end(); 
    gps_port.begin(115200);
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
    byte packet8[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x08};
    sendPacket(packet8, sizeof(packet8)); 
  }

if (c == 2) {
    byte packet8[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85, 0x2A};
    sendPacket(packet8, sizeof(packet8)); 
  }

if (c == 4) {
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

  gps_port.addMemoryForRead(serial2bufferRead, sizeof(serial2bufferRead));
   
}

void sendPacket(byte *packet, byte len){
    for (byte i = 0; i < len; i++) { gps_port.write(packet[i]); }
}

void newfile() { 
  
  dtostrf(time_number, 1, 0, sdnamebuff); 
  sprintf(namebuff, "%s.txt", sdnamebuff);
    
  if (!SD.begin(chipSelect)) { sd_ok = false; }
  else { 
    sd_ok = true; 
    SdFile::dateTimeCallback(dateTime);
    dataFile = SD.open(namebuff, FILE_WRITE);
    delay(10); 
    if (dataFile) {  
      dataFile.println("time (ms), Packet_Count (text), Mode (text), GPS_Ok (text), COG_Ok (text), FailSafe (text), GPS_Stab (text), Baro_Stab (text), Deployed (text), Wing_Opened (text), Initialised (tex), Vbatt (V), Loopmin (µS), Loopmax (µS), Loopmean (µS), GPS-date, GPS-time, lat (deg), lon (deg), alt (m), CoG (deg), Speed (m/s), Sat_in_use (text), HDOP (text), Position_Age (text), Fix_type (text), Baro_Alt (m), Vertical_Speed (m/s), Altitude (m), Baro_Weight (text), GPS_Weight (text), SetPoint_Home (deg), Err_Home (deg), Rate_Error (deg), Next_Cog (deg), Cmd_mult (text), LatB (deg), LonB (deg), Ch 0 (µs), Ch 1 (µs), Ch 2 (µs), Ch 3 (µs), Ch 4 (µs), Ch 5 (µs), Ch 6 (µs), PWM_L (µs), PWM_R (µs), PWM_D (µs)");
      dataFile.close();
      EEPROM.put(90, time_number);
    }
  }
}

void sim() { 
  sim_cmd_time = (millis()*(PI/4000));
  sim_cmd = sin(sim_cmd_time); 
  sim_cmd = map(sim_cmd, -1, 1, -180, 180); 
}

void eppclear() { 
  for ( unsigned int i = 0 ; i < EEPROM.length() ; i++ )
  EEPROM.write(i, 0); 
}

void getconfig() {

  if (!SD.begin(chipSelect)) { sd_ok = false; }
  else {
    File configFile = SD.open("config.txt", FILE_READ);
    if (configFile) {

      // Reading the baro adress 
      String memory = ""; 
      while (configFile.available()) {
        char a = configFile.read(); 
        if (a != 44) { memory = memory + a; }
        else { break; }
      }

      baro_adress = memory.toFloat();

      char a = '0'; 
      unsigned int i(0); 

      // Reading the Waypoints 
      while (configFile.available() and (i<15)) {
        String waypoint_str = ""; 
        do {
          a = configFile.read(); 
          if (a != 44) { waypoint_str = waypoint_str + a; }
          else { waypoint[i].latitude = (waypoint_str.toFloat()); break; }
        } while (a != 44); 
        waypoint_str = ""; 
        do {
          a = configFile.read(); 
          if (a != 44) { waypoint_str = waypoint_str + a; }
          else { waypoint[i].longitude = (waypoint_str.toFloat()); i++; break; }
        } while (a != 44); 
      }
      last_waypoint_number = i+1; 
      configFile.close();
    }
  }  
}



void navigation() {
  if (nav_waypoint == true) {
    if (TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),lat_B,lon_B)<waypoint_threshold) { 
      if (waypoint_number < 15) { waypoint_number++; } 
      if ((waypoint[waypoint_number].latitude !=0) and (waypoint[waypoint_number].longitude !=0)) {
        lat_B = waypoint[waypoint_number].latitude; 
        lon_B = waypoint[waypoint_number].longitude; 
      }
    }   
  }
}
