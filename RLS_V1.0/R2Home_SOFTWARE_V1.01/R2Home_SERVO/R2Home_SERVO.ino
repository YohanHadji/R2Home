#include <Servo.h>
#include <ServoInput.h>

const int InputPinServo = 2;
const int InputPinCmd = 10;
const int OutputPin = 3; 

ServoInputPin<InputPinServo> inputServo;
ServoInputPin<InputPinCmd> inputCmd;

Servo outputServo;

float parallax_steer = 1500; 
float feedback_value = 1500; 

unsigned long tread = 0; 
unsigned long tparallax = 0; 

float Kp = 1.5;
float Kd = 3;
float cumerror = 0; 
float lasterror = 0;  

float error = 0; 
float PIDsum = 0; 
float cmd = 0; 
float raterror = 0; 


void setup() {
 
 Serial.begin(57600); 
 outputServo.attach(OutputPin);

}

void loop() {
  
  getcmd(50); 
  updatecmd(200); 


}

void getcmd(int a) { 

if ((millis()-tread)>=(1000/a)) {

  tread = millis(); 

  parallax_steer = inputCmd.getPulse();
  parallax_steer = constrain(parallax_steer, 1000, 2000); 

  Serial.print(parallax_steer); Serial.print(","); Serial.print(feedback_value); Serial.print(","); Serial.print(error);  Serial.print(",");  Serial.println(PIDsum);  
 
  }
}

void updatecmd(int a) { 
  
  if ((millis()-tparallax)>=(1000/a)) { 

      tparallax = millis(); 
      
      feedback_value = inputServo.getPulseRaw();
      
      if ((feedback_value <= 0) or (feedback_value >= 2000)) { feedback_value = 500; }
      
      feedback_value = map(feedback_value, 0, 1000, 2050, 950); 
   
      error = (parallax_steer - feedback_value); 
      raterror = (error-lasterror);
      PIDsum = (Kp*error)+(Kd*raterror);  
      cmd = 1500 + PIDsum; 
      if (cmd >1500) {cmd = cmd+20;}
      if (cmd <1500) {cmd = cmd-20;}

      constrain(cmd, 1050, 1950); 
 
      outputServo.writeMicroseconds(cmd);
      
      lasterror = error; 
    
  } 
}
