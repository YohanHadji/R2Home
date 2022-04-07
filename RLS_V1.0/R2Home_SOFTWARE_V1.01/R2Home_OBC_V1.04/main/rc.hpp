#include <SBUS.h>
#include "config.h"

SBUS rx(RX_PORT);
uint16_t channels[16];
bool failsafe;
bool lostFrame;

void rc_setup() {
  rx.begin();
  if (DEBUG) { Serial.println("RC was set correctly"); } 
}

void get_rc() {
  rx.read(&channels[0], &failsafe, &lostFrame);
  
  if ((channels[3])>=1500 or (channels[3])==0) { failsafe = true; }
  else { failsafe = false; } 
  if (DEBUG) { Serial.println("Just got RC (even if no rx is connected)"); }  
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
  
