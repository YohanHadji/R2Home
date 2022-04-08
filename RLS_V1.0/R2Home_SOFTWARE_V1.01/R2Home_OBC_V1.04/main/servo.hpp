#include "config.h"
#include "control.hpp"
#include <PWMServo.h>

PWMServo steer;
PWMServo deploy;
PWMServo aux_deploy;
PWMServo left; 
PWMServo right; 
PWMServo esc; 

int servo_left  = 0;
int servo_right = 0;
int servo_aux   = 0; 
int servo_aux_deploy = 0; 

unsigned long tpwm = 0; 
 
struct servo_cmd {
  float left = 1500;
  float right = 1500; 
  float aux = 1500; 
  float aux_deploy = 1500; 
};

void servo_setup() {
  left.attach(6, 1000, 2000);  
  right.attach(7, 1000, 2000); 
  if (DROP == true) { deploy.attach(8, 1000, 2000); } 
  else              { esc.attach(8, 1000, 2000);    }
  aux_deploy.attach(9, 1000, 2000); 
}

servo_cmd cmpt_servo(uint16_t channels[16], int autopilot, int flight_mode, bool deployed, bool failsafe, bool cog_ok, bool spiral, bool separation) {
  
  float roll  = map(channels[0], 67, 1982, 1000, 2000);
  float pitch = map(channels[1], 67, 1982, 1000, 2000);
  float aux =   map(channels[2], 67, 1982, 1000, 2000);
  float sw =    map(channels[6], 67, 1982, 1000, 2000);
  
  roll  = constrain(roll, 1000, 2000); 
  pitch = constrain(pitch, 1000, 2000); 
  aux   = constrain(aux, 1000, 2000); 
  sw   = constrain(sw, 1000, 2000);

  servo_cmd steering_cmpt ;
  
  switch(flight_mode) {  
    case 0: 
    case 1: 
    case 6: 
    case 8: 
    // ---------- Stage 1 - RC mode ---------- // 
    switch (RC_MODE) {
          
      case 0: // roll only 
      steering_cmpt.right = roll; 
      steering_cmpt.left = 3000-roll; 
      break; 

      case 1: // pitch and roll mixed 
      steering_cmpt.right = roll; 
      steering_cmpt.left = 3000-roll;
      steering_cmpt.left  = steering_cmpt.left+(pitch-1500); 
      steering_cmpt.right = steering_cmpt.right+(pitch-1500); 
      break; 

      case 2: // pitch and roll separated
      steering_cmpt.left = roll; 
      steering_cmpt.right = pitch;
      break; 
    }
    if (failsafe) {
      steering_cmpt.left = 1500; 
      steering_cmpt.right = 1500; 
    }
    break; 

    case 9:
    case 10:
    case 5: 
    if (cog_ok and !spiral) {
      steering_cmpt.right = autopilot; 
      steering_cmpt.left = 3000-autopilot; 
    }
    else {
      steering_cmpt.right = 1500; 
      steering_cmpt.left = 1500; 
    }
    break; 

    case 11: 
    steering_cmpt.right = 1500; 
    steering_cmpt.left = 1500; 
    break; 
    
  }

  // ---------- Stage 2 - Linear mode ---------- // 
  switch (LINEAR_MODE) {
          
    case 0: // control is fully linear 
    break; 

    case 1: // control start at servo_start with an offset 
    if (steering_cmpt.left>=(1500+TRIG)) { 
        steering_cmpt.left = map(steering_cmpt.left, 1500, 2000, 1500+LEFT_OFFSET, SERVO_MAX_M);
    }
    else if (steering_cmpt.left<=(1500-TRIG)) {
        steering_cmpt.left = map(steering_cmpt.left, 1000, 1500, 1000, 1500-LEFT_OFFSET);
    }
    if (steering_cmpt.right>=(1500+TRIG)) { 
        steering_cmpt.right = map(steering_cmpt.right, 1500, 2000, 1500+RIGHT_OFFSET, SERVO_MAX_M);
    }
    else if (steering_cmpt.right<=(1500-TRIG)) {  
        steering_cmpt.right = map(steering_cmpt.right, 1000, 1500, 1000, 1500-RIGHT_OFFSET);
    }
    break; 
  }

  // ---------- Stage 3 - Control mode ---------- // 
  switch (CONTROL_MODE) {
          
    case 0: // neutral is center 
    break; 

    case 1: // neutral is hands up 
    steering_cmpt.right = constrain(map(steering_cmpt.right, 1500, 2000, 1000, SERVO_MAX_M), 1000, SERVO_MAX_C);
    steering_cmpt.left = constrain(map(steering_cmpt.left, 1500, 2000, 1000, SERVO_MAX_M), 1000, SERVO_MAX_C);
    break; 
  }

// -------------------------- Deployment Servo and ESC -------------------------- //

  switch(flight_mode) { 
    
    case 0: 
    case 1:
    case 3:
    case 4:
    case 5: 
    case 6:
    case 7: 
      if (deployed == true) { 
        steering_cmpt.aux = 1000; 
      }
      else {
        if (failsafe == true) { 
          steering_cmpt.aux = 2000; 
        } 
        else { steering_cmpt.aux = sw; }
      } 
    break; 

    case 2:
      if (separation == true) {
        steering_cmpt.aux_deploy = 1000;
      }
      else {
        steering_cmpt.aux_deploy = 2000;
      }
    break;

    case 8:
      steering_cmpt.aux = aux;
    break; 

     case 9:
     case 10:  
      steering_cmpt.aux = 1000; 
     break; 

     case 11:
     steering_cmpt.aux = 1000;
     break;  
  } 
  
  servo_left = steering_cmpt.left; 
  servo_right = steering_cmpt.right; 
  servo_aux = steering_cmpt.aux; 
  servo_aux_deploy = steering_cmpt.aux_deploy; 
  
  return steering_cmpt;  
}

void update_servo_cmd(servo_cmd steering_apply, unsigned int a) {
   
  if ((millis()-tpwm)>=(1000/a)) {  
    tpwm = millis(); 
    left.write(map(steering_apply.left, 1000, 2000, 0, 180));
    right.write(map(steering_apply.right, 1000, 2000, 0, 180)); 
    deploy.write(map(steering_apply.aux, 1000, 2000, 0, 180));
    aux_deploy.write(map(steering_apply.aux_deploy, 1000, 2000, 0, 180));
  }
 
}

String servo_text() {
  String left_text  = String(servo_left); 
  String right_text = String(servo_right); 
  String aux_text   = String(servo_aux); 
  String aux_deploy_text = String(servo_aux_deploy); 
  return left_text+","+right_text+","+aux_text+","+aux_deploy_text; 
}
