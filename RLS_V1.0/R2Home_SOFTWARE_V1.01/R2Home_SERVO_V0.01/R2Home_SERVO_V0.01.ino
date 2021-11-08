#include <Servo.h>
#include <movingAvg.h>

movingAvg st(5); 
Servo steer;

#define feedback_pin 2
#define cmd_pin 10

float parallax_steer = 1500; 
float feedback_value = 0;

const float Kp = 0.8;
const float Kd = 0.03;
float cumerror = 0; 
float lasterror = 0;  

unsigned long tparallax = 0; 
unsigned long tread = 0; 

void setup() {

  st.begin();
  steer.attach(3, 1000, 2000);

}

void loop() {

  Serial.begin(57600); 

  getcmd(50); 
  updatecmd(200); 

}

void getcmd(int a) {

 if ((millis()-tread)>=(1000/a)) { 
  parallax_steer = pulseIn(10, HIGH, 2100); 
 }

 if ((parallax_steer <= 999) or (parallax_steer >= 2001)) { 
  parallax_steer = 1500; 
 }

 Serial.println(parallax_steer); 

}

 
void updatecmd(int a) { 
  
  if ((millis()-tparallax)>=(1000/a)) { 

      tparallax = millis(); 
      
      feedback_value = (((pulseIn(feedback_pin, HIGH, 1200))-30)/1.050); 

      //Serial.println(feedback_value); 
      
      if ((feedback_value < 0) or (feedback_value > 2000)) { feedback_value = 500; }
      
      feedback_value = st.reading(feedback_value); 
      feedback_value = map(feedback_value, 0, 1000, 1000, 2000); 
   
      float error = (parallax_steer - feedback_value); 
      float raterror = ((error-lasterror)/(1/a));
      float PIDsum = ((Kp*error)+(Kd*raterror)); 
      float cmd = 1500 + PIDsum; 

      constrain(cmd, 1000, 2000); 
      cmd = map(cmd, 1000, 2000, 85,95);
       
      lasterror = error; 
    
      steer.write(cmd);
    
  } 
}
