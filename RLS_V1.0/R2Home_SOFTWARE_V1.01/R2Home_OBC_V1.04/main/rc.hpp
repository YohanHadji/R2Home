#include <SBUS.h>
#include "config.h"

SBUS rx(RX_PORT);
uint16_t channels[16];
bool failsafe;
bool lostFrame;

unsigned long last_rc(0); 

void rc_setup() {
  rx.begin();
  if (DEBUG) { Serial.println("RC was set correctly"); } 
}

void get_rc() {
     
    rx.read(&channels[0], &failsafe, &lostFrame);    
    if (channels[3]==0) { failsafe = true; }
    //else { failsafe = false; } 
    
    if (DEBUG) { Serial.println("Just got RC (even if no rx is connected)"); }  

    /* 
    Serial.print(channels[0]); Serial.print(","); 
    Serial.print(channels[1]); Serial.print(","); 
    Serial.print(channels[2]); Serial.print(","); 
    Serial.print(channels[4]); Serial.print(",");
    Serial.print(channels[5]); Serial.print(",");
    Serial.println(channels[6]);
    */
}

String rc_text() {
  String a_text = String(channels[0]);
  String b_text = String(channels[1]);
  String c_text = String(channels[2]);
  String d_text = String(channels[3]);
  String e_text = String(channels[4]);
  String f_text = String(channels[5]);
  String g_text = String(channels[6]);

  return a_text+","+b_text+","+c_text+","+d_text+","+e_text+","+f_text+","+g_text;
}
  
