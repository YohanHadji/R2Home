#include <PID_v1.h>
#include "config.h" 

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,NKP, NKI, NKD, DIRECT);

void navigation_setup() {
  myPID.SetTunings(NKP, NKI, NKD);
  myPID.SetOutputLimits(-180, 180);
  myPID.SetMode(MANUAL);
}

float cmpt_cmd(float err) {      
  Input = -err; 
  Setpoint = 0; 
  myPID.Compute();
  return map(Output, -180, 180, 1000, 2000); 
}
