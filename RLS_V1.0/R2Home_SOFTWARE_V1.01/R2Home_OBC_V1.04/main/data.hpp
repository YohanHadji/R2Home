#include <SD.h>
#include <Watchdog.h>
#include "feedback.hpp"

Watchdog watchdog; 
bool sd_ok = false; 
int delaySD = 100;    // Datalog 
int delayTLM = 1000;   // Tlm 

char sdnamebuff[20]; 
String mainSD;
String mainTLM;   
String minSD; 

File dataFile; 
File configFile; 
const int chipSelect = BUILTIN_SDCARD;
char namebuff[20]; 
unsigned int addr = 0;
String filename; 
int datatest = 0; 
int count = 0; 
int packet_count = 0; 

unsigned long tlm = 0;
unsigned long sd = 0; 

int time_number = 0; 


void cmpt_data_rate(int flight_mode) {
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
    if (LOW_RATE) { delaySD = 200; }
    else { delaySD = 50; }
    delayTLM = 1000;  
    break; 
  }
}

void send_data() { 
  packet_count = (packet_count +1); 
  TLM_PORT.println(mainTLM); 
  if (TLM_MONITOR) {  
    Serial.println(mainTLM);  
  }
}

void save_data(bool initialised) { 
  if (initialised == true) {
    if (sd_ok == true and SD_WRITING == true) {
      dataFile = SD.open(namebuff, FILE_WRITE);
      dataFile.println(mainSD); 
      dataFile.close();
    }
  } 
}
    
void cmpt_string_data(int flight_mode, bool initialised, bool deployed, bool wing_opened) {

  time_number = ((gps.date.day()*1000000) + (gps.time.hour()*10000) + (gps.time.minute()*100) + gps.time.second());  

  int actual_time = millis();
  String time_text = String(actual_time);
  String flight_mode_text = String(flight_mode);
  String initialised_text = String(initialised);
  String deployed_text = String(deployed);
  String wing_opened_text = String(wing_opened);
  String gps_ok_text = String(gps_ok) ;
  String cog_ok_text = String(cog_ok);
  String failsafe_text = String(failsafe);
  String vbatt_text = String(vbatt,2);
  String loop_rate_text = String(loop_rate);
  String packet_count_text = String(packet_count);
  
  String status_text = time_text+","+packet_count_text+","+flight_mode_text+","+initialised_text+","+deployed_text+","+wing_opened_text+","+gps_ok_text+","+cog_ok_text+","+failsafe_text+","+vbatt_text+","+loop_rate_text;  
  
  mainSD = status_text+","+gps_text()+","+baro_text()+","+nav_text()+","+rc_text()+","+servo_text();
  mainTLM = "/*"+status_text+","+gps_text()+","+baro_text()+","+servo_text()+"/*";
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
      dataFile.println("time (ms), Packet_Count (text), Mode (text), Initialised (text), Deployed (text), Wing_Opened (text), GPS_Ok (text), COG_Ok (text), FailSafe (text), Vbatt (V), Loop_rate (Hz), GPS-date, GPS-time, lat (deg), lon (deg), alt (m), CoG (deg), Speed (m/s), Sat_in_use (text), HDOP (text), Position_Age (text), Fix_type (text), Baro_Alt (m), Baro_Vspeed (m/s), Altitude (m), Baro_Weight, GPS_Weight, Baro_Vspeed_AVG (m/s), GPS_Vspeed_AVG (m/s), VDOWN (m/s), SetPoint_Home (deg), Err_Home (deg), LatB (deg), LonB (deg), WaypointNumber (text), Distance (m), Ch 0 (us), Ch 1 (us), Ch 2 (us), Ch 3 (us), Ch 4 (us), Ch 5 (us), Ch 6 (us), PWM_L (us), PWM_R (us), PWM_D (us)");
      dataFile.close();
    }
  }
}


void getconfig() {
  watchdog.enable(Watchdog::TIMEOUT_1S);

  if (!SD.begin(chipSelect)) { sd_ok = false; delay(1500); }
  else {
    sd_ok = true; 
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
      while (configFile.available() and (i<16)) {
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
          else { waypoint[i].longitude = (waypoint_str.toFloat()); break; }
        } while (a != 44); 
        waypoint_str = "";
        do {
          a = configFile.read(); 
          if (a != 44) { waypoint_str = waypoint_str + a; }
          else { waypoint[i].radius = (waypoint_str.toFloat()); i++; break; }
        } while (a != 44); 
      }
      last_waypoint_number = i+1; 
      configFile.close();
    }
    else { 
      delay(1500) ;
    }
  }  
}
