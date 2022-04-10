#include "config.h"
#include "position.hpp"
#include "servo.hpp"

struct gps_location { 
  double latitude = 0; 
  double longitude = 0; 
  double radius = 0; 
};

gps_location waypoint[17]; 
gps_location current_waypoint; 

int waypoint_number = 0; 
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

float cmpt_setpoint(gps_location waypoint) {                                                
  return TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),waypoint.latitude,waypoint.longitude);                                      
}

float cmpt_error(float cog, float setPoint) {
  return getangle(cog, setPoint); 
}

void navigation() {
   
  if (NAV_WAYPOINT == true) {
    
    float distance_to = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),current_waypoint.latitude,current_waypoint.longitude);
     
    if (distance_to < current_waypoint.radius) { 
      
      if (waypoint_number < 15) { 
        waypoint_number++;
      } 
      
      if ((waypoint[waypoint_number].latitude !=0) and (waypoint[waypoint_number].longitude !=0)) {
        current_waypoint = waypoint[waypoint_number]; 
      }  
    }   
  } 
}

String control_text() {
  String setPoint_Home_text = String(cmpt_setpoint(current_waypoint),2);
  String errHome_text = String(cmpt_error(gps.course.deg(), cmpt_setpoint(current_waypoint)),2); 
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
