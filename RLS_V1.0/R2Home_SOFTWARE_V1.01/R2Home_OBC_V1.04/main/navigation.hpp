#include "config.h"
#include "position.hpp"
#include "servo.hpp"
#include "where_to_go.hpp"

int last_waypoint_number = 0; 

float getangle(float a, float b) { 
  float angle = 0; 
  if (abs(a-b) < 180) { angle = (b-a);}
  else { 
    if ((a-b) < 0) { angle = (-360) +(b-a);}
    else { angle = (360) + (b-a);}
  }
  return angle;    
}

float cmpt_setpoint(float gps_latitude, float gps_longitude, gps_location waypoint) {                                                
  return TinyGPSPlus::courseTo(gps_latitude, gps_longitude, waypoint.latitude, waypoint.longitude);                                      
}

float cmpt_error(float cog, float setPoint) {
  return getangle(cog, setPoint); 
}

String control_text() {
  String setPoint_Home_text = String(cmpt_setpoint(gps.location.lat(), gps.location.lng(), current_waypoint),2);
  String errHome_text = String(cmpt_error(gps.course.deg(), cmpt_setpoint(gps.location.lat(), gps.location.lng(), current_waypoint)),2); 
  String lat_B_text = String(current_waypoint.latitude,5);
  String lon_B_text = String(current_waypoint.longitude,5);
  String waypoint_text = String(waypoint_number); 
  String waypoint_distance = String(TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),current_waypoint.latitude,current_waypoint.longitude));
  String waypoint_threshold = String(current_waypoint.radius); 
  
  return setPoint_Home_text+","+errHome_text+","+lat_B_text+","+lon_B_text+","+waypoint_text+","+waypoint_distance+","+waypoint_threshold; 
}

String nav_text() {
  return pos_text()+","+control_text(); 
}
