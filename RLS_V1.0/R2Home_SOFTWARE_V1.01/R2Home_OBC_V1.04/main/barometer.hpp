#include "config.h"
#include <Adafruit_BMP280.h>
#include <movingAvg.h>

Adafruit_BMP280 bmp(&Wire);
int baro_adress = 0x00; 
double baro_set  = 1000.0; 

float alt_baro      = 0;
float pressure_baro = 0; 
float prev_alt_baro = 0; 
float baro_vspeed   = 0; 
float baro_count    = 0; 

bool new_baro       = false;
bool stable_descent = false; 

movingAvg b_al(BARO_AL_AVG); 
movingAvg b_vs(BARO_VS_AVG); 
movingAvg ps(BARO_PS_AVG); 

unsigned long baroA     = 0; 
unsigned long baroB     = 0; 
unsigned long baro_blk  = 0;
unsigned long baro_clb  = 0; 

void baroset(float alt_set, int factor) { 
  do {
      if ((micros()-baro_blk)>=10) {
        baro_blk = millis();    
        alt_baro = bmp.readAltitude(baro_set);
        baro_set = (baro_set + ((alt_set-alt_baro)/(8*factor)));
        prev_alt_baro = alt_baro; 
        }
     } while (abs(alt_baro-alt_set)>0.01); 
}

void baro_adjust(float alt_set, int factor) {
  if ((millis()-baro_clb)>1000) {
    baro_clb = millis();
    alt_baro = bmp.readAltitude(baro_set);
    baro_set = (baro_set + ((alt_set-alt_baro)/(8*factor))); 
  }
}

void barometer_setup() {
  b_vs.begin(); 
  b_al.begin();
  ps.begin(); 
  bmp.begin(baro_adress);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
  Adafruit_BMP280::SAMPLING_X1,   
  Adafruit_BMP280::SAMPLING_X2,    
  Adafruit_BMP280::FILTER_X16,      
  Adafruit_BMP280::STANDBY_MS_1);
  baroset(0, 1);  
}

void cmpt_vertical_speed_baro(float da, int dt) {  
  baro_vspeed  = (da/(dt/1000.0));
  baro_vspeed = b_vs.reading(baro_vspeed*100); 
  baro_vspeed = (baro_vspeed/100);
}

void get_baro(int mode) {
  if ((millis()-baroA)>=10 and mode == 0) { 
    
    baroA = millis();
    unsigned waitd = millis(); 
    
    alt_baro = bmp.readAltitude(baro_set);
    pressure_baro = bmp.readPressure(); 
        
    alt_baro = (b_al.reading(alt_baro*100.0)/100.0);
    pressure_baro = (ps.reading(pressure_baro*100.0)/100.0);
    
    if (((millis() - waitd) >= 100)) { 
      barometer_setup(); 
    } 
    baro_count = (baro_count + 1);;
     
    if (baro_count >= BARO_VS_SAMPLE) { 
      baro_count = 0; 
      cmpt_vertical_speed_baro(alt_baro-prev_alt_baro, baroA-baroB); 
      baroB = millis(); 
      prev_alt_baro = alt_baro; 
      new_baro = true;
    } 
  }

  else {
    baroA = millis();
    unsigned waitd = millis(); 
    
    alt_baro = bmp.readAltitude(baro_set);
    pressure_baro = bmp.readPressure(); 
        
    alt_baro = (b_al.reading(alt_baro*100.0)/100.0);
    pressure_baro = (ps.reading(pressure_baro*100.0)/100.0);
    
    if (((millis() - waitd) >= 100)) { 
      barometer_setup(); 
    } 
    baro_count = (baro_count + 1);;
     
    if (baro_count >= BARO_VS_SAMPLE) { 
      baro_count = 0; 
      cmpt_vertical_speed_baro(alt_baro-prev_alt_baro, baroA-baroB); 
      baroB = millis(); 
      prev_alt_baro = alt_baro; 
      new_baro = true;
    } 
  }
}


String baro_text() {
  String alt_baro_text = String(alt_baro, 3);
  String baro_vspeed_text = String(baro_vspeed, 3);
  String baro_pressure_text = String(pressure_baro, 2); 
  return alt_baro_text+","+baro_pressure_text+","+baro_vspeed_text; 
}
