#include <EasyBuzzer.h>
#include <Adafruit_NeoPixel.h>

#include "config.h"
#include "battery.hpp"
#include "camera.hpp"
#include "debug.hpp"
#include "navigation.hpp" 
#include "rc.hpp"
#include "servo.hpp" 

#define LED_PIN     3
#define LED_COUNT  1
#define BRIGHTNESS 15

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800); 
int duration = 0; 
int timeled = 0; 
int lastled = 0; 

int BUZZER_PIN = 2;
int mid_freq = 0; 
float time_sweep = 0; 
int direc = 0;
unsigned long sweep_start = 0; 
unsigned long tbeep = 0; 
bool new_sweep = false; 
bool current_sweep = false;
int current_freq = 0; 
float sweep_step = 1;
float time_step = 1; 
unsigned long last_waypoint_time = 0; 
unsigned long sat_buzz = 0;
unsigned long batt_buzz = 0;
unsigned long t_turn = 0; 

int beep_start = 0; 
int beep_stop = 0; 



void buzzer_setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  EasyBuzzer.setPin(BUZZER_PIN);
  tone(BUZZER_PIN, 523); 
  delay(200); 
  tone(BUZZER_PIN, 582); 
  delay(200); 
  tone(BUZZER_PIN, 762); 
  delay(200); 
  noTone(BUZZER_PIN);
}

void buzzer_end_setup() {
  tone(BUZZER_PIN, 582); 
  delay(200); 
  tone(BUZZER_PIN, 830); 
  delay(300); 
  noTone(BUZZER_PIN);
}

void sweep_beep_set(int freq, int dur, int dir) {
  mid_freq = freq; 
  time_sweep = dur; 
  new_sweep = true; 
  direc = dir; 
  beep_start = millis(); 
}

void buzzer_turn(int flight_mode, int turn_cmd) {
    if (BUZZER_TURN == true and (flight_mode == 5 or flight_mode == 9 or flight_mode == 10)) { 

    if (millis()-last_waypoint_time >= 2000) {
    
      int tone_turn = map(turn_cmd, 1000, 2000, 1000, 2000); 
     
      if (BUZZER_SWEEP == true) {
        double t_down    = abs(map(turn_cmd, 1000, 2000, -3, 3))+1; 
        t_down = (1/t_down)*1500;
        if ((millis()-t_turn)>=t_down) { 
          t_turn = millis(); 
          int dir = 0; 
          if (tone_turn >= 1500) { dir = 0; }
          else                  { dir = 1; }
          float force = 0; 
          if (tone_turn >= 1500) { force = tone_turn - 1500; } 
          else                  { force = 1500 - tone_turn; }
          force = constrain(force, 10, 200); 
          sweep_beep_set(tone_turn, force, dir); 
       }
     }
     else {
      double t_down    = abs(map(turn_cmd, 1000, 2000, -5, 5))+1; 
       t_down = (1/t_down)*500;
        if ((millis()-t_turn)>=t_down) {
          t_turn = millis(); 
          EasyBuzzer.singleBeep(tone_turn, 20); 
        }
      } 
    }
  } 
}

void sweep_beep_update() {
  if (direc == 0) {
    if (new_sweep == true) { 
      sweep_start = millis(); 
      new_sweep = false; 
      current_sweep = true; 
      current_freq = mid_freq-((sweep_step*time_sweep)/(2*time_step)); 
      EasyBuzzer.singleBeep((current_freq),time_step);
    }
  
    if ((current_sweep == true) and ((millis()-sweep_start)>=(time_step)) and (current_freq<mid_freq+((sweep_step*time_sweep)/(2*time_step)))) {
      sweep_start = millis(); 
      current_freq = current_freq + sweep_step;  
      EasyBuzzer.singleBeep((current_freq),time_step);
      beep_stop = millis(); 
    }
  }
  else {
    if (new_sweep == true) { 
      sweep_start = millis(); 
      new_sweep = false; 
      current_sweep = true; 
      current_freq = mid_freq+((sweep_step*time_sweep)/(2*time_step)); 
      EasyBuzzer.singleBeep((current_freq),time_step);
    }
  
    if ((current_sweep == true) and ((millis()-sweep_start)>=(time_step)) and (current_freq>mid_freq-((sweep_step*time_sweep)/(2*time_step)))) {
      sweep_start = millis(); 
      current_freq = current_freq - sweep_step;  
      EasyBuzzer.singleBeep((current_freq),time_step);
      beep_stop = millis(); 
    }
  } 
}

void buzzer_batt() {
  cells = int(vbatt/3.65); 

  if ((vbatt < (cells*BCRITICAL)) and (vbatt > NO_BATT)) { 
    batt_critical = true; 
    if (millis()-batt_buzz>=100) { 
      batt_buzz = millis(); 
      EasyBuzzer.singleBeep(3000,25);
    }
  }
  
  else { batt_critical = false; }

  if ((vbatt < (cells*BLOW)) and (vbatt > (cells*BCRITICAL))) {  
    batt_low = true; 
    if (millis()-batt_buzz>=200) { 
      batt_buzz = millis(); 
      EasyBuzzer.singleBeep(3000,50);
    }
  }

  else { batt_low = false; }
}

void update_buzzer() {
  sweep_beep_update();
  EasyBuzzer.update();
}

void led_setup() { 
  strip.begin();           
  strip.show();            
  strip.setBrightness(255);
}

void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { 
    strip.setPixelColor(i, color);         
    strip.show();                          
  }
}

void setled(int a, int b, int c, int d, int e) { 
 
  if (batt_critical == true) { a = 255; b = 0; c = 0; d = 25; e = 50; }
  if (batt_low == true) { a = 255; b = 0; c = 0; d = 25; e = 200; }

  if ((millis()-lastled)>=e) { 
    lastled = millis(); 
    if (LED_MODEL == 1) { colorWipe(strip.Color(b,a,c), 0); }
    else { colorWipe(strip.Color(a,b,c), 0); }
    duration = d; 
    timeled = millis(); 
 }
}

void updateled() { 
  if ((millis()-timeled)>=duration) { 
    colorWipe(strip.Color(0,0,0), 0);
 }  
}
