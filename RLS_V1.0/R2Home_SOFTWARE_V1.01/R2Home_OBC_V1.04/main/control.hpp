#include <PID_v1.h>
#include "config.h" 

int SERVO_MAX_M_W = 0; 
int SERVO_MAX_C_W = 0; 

int SERVO_MAX_M = 0; 
int SERVO_MAX_C = 0; 

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,NKP, NKI, NKD, DIRECT);

void navigation_setup() {
  myPID.SetTunings(NKP, NKI, NKD);
  myPID.SetOutputLimits(-180, 180);
  myPID.SetMode(MANUAL);
  if (DEBUG) { Serial.println("PIDs set correctly"); } 
}

float cmpt_cmd(float err) {      
  Input = -err; 
  Setpoint = 0; 
  myPID.Compute();
  return map(Output, -180, 180, 1000, 2000); 
}

void cmpt_weight_gain() {
  if (AUTO_GAIN_WEIGHT) {
    int total_weight = SYSTEM_WEIGHT+PAYLOAD_WEIGHT;
    total_weight = constrain(total_weight, 500, 1500); 
    SERVO_MAX_M_W = map(total_weight, 500, 1500, 2000, 1500); 
    SERVO_MAX_C_W = map(total_weight, 500, 1500, 1500, 1250); 
  }
  else {
    SERVO_MAX_M_W = SERVO_MAX_M_DEF;
    SERVO_MAX_C_W = SERVO_MAX_C_DEF;
  }
}

void cmpt_pressure_gain(float pressure_ratio) {
  if (AUTO_GAIN_PRESSURE) {
    SERVO_MAX_M = map((SERVO_MAX_M_W-1000)/pressure_ratio, 0, 1000, 1000, 2000); 
    SERVO_MAX_C = map((SERVO_MAX_M_W-1000)/pressure_ratio, 0, 1000, 1000, 1500);
  }
  else {
    SERVO_MAX_M = SERVO_MAX_M_W;
    SERVO_MAX_C = SERVO_MAX_C_W; 
  }
}

String servo_max_cmd_text() {
  String max_m_w_text  = String(SERVO_MAX_M_W); 
  String max_c_w_text = String(SERVO_MAX_C_W); 
  String max_m_text   = String(SERVO_MAX_M); 
  String max_c_text   = String(SERVO_MAX_C); 
  return max_m_w_text+","+max_c_w_text+","+max_m_text+","+max_c_text ; 
}
