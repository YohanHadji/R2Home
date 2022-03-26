#include "config.h"
#include "data.hpp"

int flight_mode   = 0; 
int dep_altitude  = 0; 
bool initialised  = false;
bool flight_started = false;
bool deployed     = false; 
bool wing_opened  = false; 
bool spiral = false;
unsigned long init_time = 0; 


//------------------- 0 -------------------//

void flight_init() { 
  
  if ((gps.satellites.value()>=6 and gps_ok and gps.hdop.value()<200) or NO_INIT) {

    EasyBuzzer.beep(3000,100,50,10,500,1);

    if ((NAV_WAYPOINT == true) and (NAV_HOME == true)) {
      waypoint[last_waypoint_number].latitude = gps.location.lat();
      waypoint[last_waypoint_number].longitude = gps.location.lng();
      waypoint[last_waypoint_number].radius = HOME_WAYPOINT_THRESHOLD; 
      current_waypoint = waypoint[waypoint_number]; 
    }

    else if ((NAV_WAYPOINT == false) and (NAV_HOME == true)) {
      current_waypoint.latitude = gps.location.lat();
      current_waypoint.longitude = gps.location.lng();
      current_waypoint.radius = HOME_WAYPOINT_THRESHOLD; 
    }

    else if ((NAV_WAYPOINT == true) and (NAV_HOME == false)) {
      current_waypoint = waypoint[waypoint_number]; 
    }

    baroset(gps.altitude.meters(), 1); 

    get_baro(1); 

    b_vs.reset(); 
    b_al.reset(); 
    g_vs.reset();

    cmpt_string_data(flight_mode, initialised, deployed, wing_opened, spiral);

    ground_altitude = gps.altitude.meters();
    dep_altitude = (DEP_ALT+ground_altitude);
  
    setcam(1, 60, 60); 
    newfile();

    if (DROP == true) { flight_mode = 1;}
    else { flight_mode = 8; }

    init_time = millis(); 
    initialised = true;      
 }

 if (millis()-sat_buzz>5000) { 
    sat_buzz = millis(); 
    EasyBuzzer.beep(2000,50,25,gps.satellites.value(),100,1); 
  } 
  
}

//------------------- 1 -------------------//

void ready_steady() { 

 if (millis()-init_time>=3000) { 

  if (is_ascent(VUP, 0))    {
    flight_mode = 2; 
    EasyBuzzer.beep(1000,100,50,2,500,1); 
    setcam(1, 20, 600); 
  }
   
  if (is_descent(v_down(VDOWN), 0)) {  
    flight_mode = 3; 
    EasyBuzzer.beep(3000,100,50,3,500,1); 
    setcam(1, 60, 600); 
    init_time = millis();
  } 
  
  if ((atof(fix_type.value()) < 2) and (NO_INIT == false) and (flight_started == false))  { 
    flight_mode = 0; 
  }

  if (I_WANT_TO_FLY) {
    flight_mode = 5; 
  }
   
 }
} 

//------------------- 2 -------------------//

void flight_ascent() { 
  if (is_descent(0, 0)) {
    flight_mode = 1;
  }  
  if ((gps.altitude.meters()-ground_altitude)>10) {
    flight_started = true; 
  }
}

//------------------- 3 -------------------//

void flight_descent() { 
  
  if (is_ascent(0, 0)) {
    flight_mode = 1; 
  } 
  
  if (is_descent(v_down(VDOWN), 0) and (merged_alt < dep_altitude or (millis()-init_time>DESCENT_TIMER))) { 
    flight_mode = 4;
    EasyBuzzer.beep(3000,100,50,5,500,1); 
    deployed = true; 
    init_time = millis(); 
    setcam(1, 60, 600); 
  }  
 }   

//------------------- 4 -------------------//

void flight_gliding() {

  if ((millis()-init_time) >= OPENING_TIMER) {
    wing_opened = true; 
    
    if (failsafe == true) {
      flight_mode = 5; 
      setcam(1, 20, 600); 
      myPID.SetMode(AUTOMATIC);
    } 
    
    else {
      flight_mode = 6; 
      myPID.SetMode(MANUAL);
    }
  } 
}

//------------------- 5 -------------------//

void flight_gliding_auto() { 

  if (!gps_ok) {
    flight_mode = 11;  
    myPID.SetMode(MANUAL);
  }

  if (is_descent(v_down(-5), 0)) {
    spiral = true; 
  }

  if (is_ascent(v_down(-2), 0)) {
    spiral = false;
  }
  
  if (failsafe == false) {
    flight_mode = 6;
    myPID.SetMode(MANUAL);
  }  

  if (I_WANT_TO_FLY == true) { 
    flight_mode = 5;
  } 
  
  navigation(); 
}

//------------------- 6 -------------------//

void flight_gliding_manual() { 
  if (failsafe == true) {
    flight_mode = 5;
    myPID.SetMode(AUTOMATIC);
  }  
}

//------------------- 7 -------------------//

void landed() {  
}

//------------------- 8 -------------------//

void motorised_man() {

  if (channels[6] > 1000) { 
    flight_mode = 9; 
    myPID.SetMode(AUTOMATIC); 
  } 
  
  if (failsafe == true) { 
    flight_mode = 10; 
    myPID.SetMode(AUTOMATIC); 
  }
}

//------------------- 9 -------------------//

void motorised_auto() { 
  
  navigation(); 
  if (channels[6] < 1000) { 
    flight_mode = 8; 
  }
}

//------------------- 10 -------------------//

void motorised_failSafe() { 
  
  navigation();
   
  if (failsafe == false) { 
    flight_mode = 8; 
    myPID.SetMode(MANUAL); 
  }
}

//------------------- 11 -------------------//

void flight_gliding_no_gps() {
  if (gps_ok) {
    flight_mode = 5; 
    myPID.SetMode(AUTOMATIC);
  }
}

//------------------- STATE MACHINE -------------------//

void cmpt_flight_state() {
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

    case 11: 
      flight_gliding_no_gps(); 
      setled(255, 0, 0, 25, 200);
    break; 
  }
}
