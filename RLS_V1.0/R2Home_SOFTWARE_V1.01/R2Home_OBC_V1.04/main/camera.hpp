#include "config.h"
 
int cam = 23;
bool camIsOn = false;
bool camReady = false; 
int onDuration = 0; 
int offDuration = 0; 

unsigned long camOn = 0; 
unsigned long camOff = 0; 

void camera_setup() { 
  pinMode(cam, OUTPUT);
  digitalWrite(cam, LOW);
}

void setcam(int a, int b, int c) {  
  switch(a) { 
    
    case 1 : 
    digitalWrite(cam, HIGH); 
    camOn = millis(); 
    onDuration = (b+4)*1000; 
    offDuration = c*1000;
    camIsOn = true; 
    break;
     
    case 0 : 
    digitalWrite(cam, LOW); 
    camOff = millis(); 
    onDuration = (b+4)*1000; 
    offDuration = c*1000; 
    camIsOn = false; 
    break;    
  }
  camReady = true; 
}

void updatecam() {
  if (CAM_CYCLE and camReady) {
    if(camIsOn) {    
      if ((millis()-camOn)>=onDuration) {
        setcam(0, (onDuration/1000)-4, offDuration/1000); 
      }
      else {
        if ((millis()-camOff)>offDuration) {
          setcam(1, (onDuration/1000)-4, offDuration/1000); 
        }  
      }
    }
  }
}
