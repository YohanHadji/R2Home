#include "config.h"
#include "barometer.hpp"
#include "gps.hpp"
#include <movingAvg.h>

float merged_alt = 0; 
float merged_vspeed = 0; 

float baro_alt_weight = 1;
float gps_alt_weight = 1; 
float baro_vspeed_weight = 1;
float gps_vspeed_weight = 1; 

float ground_altitude = 0;
unsigned long tstab = 0; 

float gps_stab_factor = 0; 
float baro_stab_factor = 0; 

bool new_baro_fusion = false;
bool new_gps_fusion = false; 

movingAvg gps_v(GPS_SAFE_AVG); 
movingAvg baro_v(BARO_SAFE_AVG);
movingAvg ps_p(PRE_PE_AVG);  

float pressure_percentage;

void position_setup() {
  gps_v.begin(); 
  baro_v.begin(); 
  ps_p.begin();
}

void cmpt_fusion() {
  if (new_baro_fusion == true and new_gps_fusion == true) {
    new_baro_fusion = false;
    new_gps_fusion = false; 
     
    pressure_percentage = (ps_p.reading((pressure_baro / (baro_set*100.0))*100.0)/100.0) ;
    baro_alt_weight = pressure_percentage;
    baro_vspeed_weight = pressure_percentage*pressure_percentage;

    double h_dop = gps.hdop.value(); 
  
    if (gps_ok) { 
      gps_alt_weight = 50.0/h_dop;
      gps_vspeed_weight = 1; 
    }
    else { 
      gps_alt_weight = 0; 
      gps_vspeed_weight = 0; 
    }
      
    merged_alt = ((alt_baro*baro_alt_weight)+(gps.altitude.meters()*gps_alt_weight))/(baro_alt_weight+gps_alt_weight); 
    merged_vspeed = ((baro_vspeed*baro_vspeed_weight)+(gps_vspeed*gps_vspeed_weight))/(baro_vspeed_weight+gps_vspeed_weight); 
    
  }
}

bool cmpt_vertical_state() {
  if (new_baro) {
    new_baro = false; 
    new_baro_fusion = true;
     baro_stab_factor = (baro_v.reading(baro_vspeed*100.0))/100.0;
  }
  if (new_gps) {
    new_gps = false; 
    new_gps_fusion = true;
    gps_stab_factor = (gps_v.reading(gps_vspeed*100))/100.0; 
  }
}

bool is_ascent(int v_trigger, bool mode) {
  if (mode == 0) {
    if (baro_stab_factor>v_trigger) {
      return true;
    }
    else {
      return false;
    }
  }
  else {
    if (gps_stab_factor>v_trigger and baro_stab_factor>v_trigger) {
      return true;
    }
    else {
      return false;
    }
  } 
}

bool is_descent(int v_trigger, bool mode) {
  if (mode == 0) {
    if (baro_stab_factor<v_trigger) {
      return true;
    }
    else {
      return false;
    }
  }
  else {
    if (gps_stab_factor<v_trigger and baro_stab_factor<v_trigger) {
      return true;
    }
    else {
      return false;
    }
  }   
}

float v_down(int vdown) {
  return sqrt((baro_set*100.0)/pressure_baro)*vdown;
}

String pos_text() {
  String merged_alt_text = String(merged_alt,3);  
  String gps_weight_text = String(gps_alt_weight,2);
  String baro_weight_text = String(baro_alt_weight,3);
  String baro_stab_text = String(baro_stab_factor, 2); 
  String gps_stab_text = String(gps_stab_factor, 2); 
  String v_down_text = String(v_down(VDOWN), 2); 
  return merged_alt_text+","+baro_weight_text+","+gps_weight_text+","+baro_stab_text+","+gps_stab_text+","+v_down_text;
}





  
