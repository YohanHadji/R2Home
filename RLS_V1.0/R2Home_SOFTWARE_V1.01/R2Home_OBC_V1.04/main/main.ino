// This code is V3.02
// Updated on April 8th

#include <movingAvg.h>
#include "flight_state.hpp" 

servo_cmd steering; 

float setPoint_waypoint = 0; 
float error_waypoint    = 0; 
float cmd_to_waypoint   = 0;

void setup() {

  Serial.begin(115200);

  if (DEBUG) { Serial.println("Just barely waking up"); } 
  
  TLM_PORT.begin(57600); 

  if (DEBUG) { Serial.println("Ready to initialise all the hardware"); } 

  battery_setup(); 
  buzzer_setup(); 
  camera_setup();
  gps_setup(57600, GPS_FREQ, 2, 1, 0); // baud, Hz, mode, nmea, cog filter (0 = Off, 1 = On)
  led_setup();
  rc_setup();
  servo_setup();
  position_setup();
  
  getconfig(); 

  watchdog.reset();
  
  barometer_setup();  
  navigation_setup(); 

  cmpt_weight_gain();

  buzzer_end_setup(); 
}

void loop() {
  
  watchdog.reset();

  getdata();  // Getting data from all the sensors 
  datacmpt(); // Using this data to do all what we have to do 
  loop_count++;     

  if ((millis()-tloop)>=1000) {
    tloop = millis(); 
    loop_rate = loop_count;
    loop_count = 0; 
  }

  if (DEBUG) {
    delay(10); 
  }
  
}

void getdata() { 
 get_gps(); 
 get_baro(0); 
 get_rc(); 
 get_vbatt(); 
 
}

void datacmpt() {

  cmpt_pressure_gain(pressure_sqrt_ratio()); 
  
  if (new_cog) {
    new_cog = false;
    if (DEBUG) { Serial.println("New direction and command computed"); }  
    setPoint_waypoint = cmpt_setpoint(current_waypoint); 
    error_waypoint    = cmpt_error(gps.course.deg(), setPoint_waypoint);
    cmd_to_waypoint   = cmpt_cmd(error_waypoint);
  }

  if (I_WANT_TO_FLY) {
    cmd_to_waypoint = sim(); 
    //Serial.println(cmd_to_waypoint); 
    cog_ok = true;
  }

  steering = cmpt_servo(channels, cmd_to_waypoint, flight_mode, deployed, failsafe, cog_ok, spiral, separation);
  update_servo_cmd(steering, SERVO_RATE); 

  cmpt_flight_state(); 
  cmpt_data_rate(flight_mode); 
  if (initialised) {
    cmpt_fusion(); 
  }
  cmpt_vertical_state(); 

  if ((millis()-sd)>=delaySD) { 
    sd = millis(); 
    cmpt_string_data(flight_mode, initialised, deployed, wing_opened, spiral);
    save_data(initialised);  
  }
  
  if (millis()-tlm>=delayTLM) { 
    tlm = millis(); 
    send_data();
  }
 
  buzzer_turn(flight_mode, setPoint_waypoint); 
  buzzer_batt(); 
  update_buzzer(); 
  updateled(); 
  updatecam(); 
 
}
