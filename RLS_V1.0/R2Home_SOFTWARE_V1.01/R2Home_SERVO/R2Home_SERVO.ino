#include <Servo.h>
#include <ServoInput.h>

// ------------------------------------------------------------------------------------ // 

bool sim_signal = true;   // Use this line to switch between simulated signal or Servo tester 

// ------------------------------------------------------------------------------------ // 

const int InputPinServo = 2;
const int InputPinCmd   = 10;
const int OutputPin     = 3; 

ServoInputPin<InputPinServo> inputServo;
ServoInputPin<InputPinCmd> inputCmd;

Servo outputServo;

float parallax_steer = 1500; 
float feedback_value = 1500; 

unsigned long tread     = 0; 
unsigned long tchange   = 0; 
unsigned long tparallax = 0; 

float Kp        = 1.5;
float Kd        = 6; 
float lasterror = 0;  

float error    = 0;
float raterror = 0; 
 
float cmd      = 1500; 
float sim_cmd  = 1500; 

float PIDsum = 0;

int toss; 

void setup() {
 
 Serial.begin(57600); 
 outputServo.attach(OutputPin);

}

void loop() {

  if (sim_signal == true) { sim(3);     }
  else                    { getcmd(50); }
  
  updatecmd(200); 
  
}

// ------------------ Get command from the onboard computer or the servo tester ------------------ // 

void getcmd(int a) { 

if ((millis()-tread)>=(1000/a)) {
  tread = millis(); 

  parallax_steer = inputCmd.getPulse();
  parallax_steer = constrain(parallax_steer, 1000, 2000); 
 
  }
}

// ------------------ Simulate a command switching every a seconds from 1000 to 2000  ------------------ //

void sim(int a) { 

  if ((millis()-tchange)>=a*1000) {
    tchange = millis();
    if (toss == 0) {
      sim_cmd = 1000; 
      toss = 1;
    }
    else {
      sim_cmd = 2000; 
      toss = 0; 
    }  
  } 
  parallax_steer = sim_cmd;
}

// ------------------ Update the command at a Hz  ------------------ //

void updatecmd(int a) { 
  
  if ((millis()-tparallax)>=(1000/a)) { 

      tparallax = millis(); 
      
      feedback_value = inputServo.getPulseRaw();
      
      if ((feedback_value <= 25) or (feedback_value >= 1100)) { feedback_value = 515; }

      constrain(feedback_value, 30, 1066); 
      feedback_value = map(feedback_value, 30, 1066, 2000, 1000); 

      // ------------------------------------------------------------------------------------ // 

      parallax_steer = map(parallax_steer, 1000, 2000, 1100, 1900); // this is the ONLY line to change to reduce the range, for example, "[...] 1450, 1550)" 

      // ------------------------------------------------------------------------------------ //
     
      error = (parallax_steer - feedback_value); 
      raterror = (error-lasterror);
      PIDsum = (Kp*error)+(Kd*raterror);  
      cmd = 1500 + PIDsum; 
        
      if (cmd >1500) {cmd = cmd+20;}
      if (cmd <1500) {cmd = cmd-20;}
        
      constrain(cmd, 1280, 1720); 
      Serial.print(parallax_steer); Serial.print(","); Serial.println(feedback_value);  
      outputServo.writeMicroseconds(cmd);
      lasterror = error;  
  } 
}
