// This file is used as an "add on", the only function defined here should be "gps_location where to go(float gps_latitude, float gps_longitude)"
// This function takes in parameter the current position, and should return a waypoint goal, which of course should be choosen based on the current position. 

#include <PID_v1.h>
#include "config.h" 

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,NKP, NKI, NKD, DIRECT);

float cmpt_cmd(float err) {      
  Input = -err; 
  Setpoint = 0; 
  myPID.Compute();
  return map(Output, -180, 180, 1000, 2000); 
}
