// This file is used as an "add on", the only function defined here should be "gps_location where to go(float gps_latitude, float gps_longitude)"
// This function takes in parameter the current position, and should return a waypoint goal, which of course should be choosen based on the current position. 

#include "config.h"
#include <TinyGPS++.h>

struct gps_location { 
  double latitude = 0; 
  double longitude = 0; 
  double radius = 0; 
};

gps_location waypoint[17]; 
gps_location current_waypoint; 

int waypoint_number = 0; 

gps_location where_to_go(float gps_latitude, float gps_longitude) {
   
  if (NAV_WAYPOINT == true) {
    
    float distance_to = TinyGPSPlus::distanceBetween(gps_latitude, gps_longitude, current_waypoint.latitude, current_waypoint.longitude);
     
      if (waypoint_number < 15) { 
        
        if (distance_to < current_waypoint.radius) { 
           
        if ((waypoint[waypoint_number+1].latitude !=0) and (waypoint[waypoint_number+1].longitude !=0)) {
          waypoint_number++;
          current_waypoint = waypoint[waypoint_number]; 
         
        }  
      }  
    }  
  }

  return current_waypoint; 

}
