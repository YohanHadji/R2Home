#include "config.h"

#include <movingAvg.h>
movingAvg voltage(50); 
 
float vpin = A17; 

float vbatt = 0; 
boolean batt_critical = false; 
boolean batt_low = false; 
int cells = 0; 

void battery_setup() {
  pinMode(vpin, INPUT); 
  voltage.begin();
}

void get_vbatt() {
  analogReadResolution(12); 
  vbatt = analogRead(A17); 
  vbatt = (voltage.reading(vbatt));
  vbatt = vbatt/223.39; 
}
