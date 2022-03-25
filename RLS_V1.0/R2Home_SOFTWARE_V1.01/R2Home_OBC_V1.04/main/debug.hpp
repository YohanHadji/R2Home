#include "config.h"

double long sim_cmd_time = 0; 
float sim_cmd   = 0; 
int loop_count  = 0;  
int loop_rate   = 999;
unsigned long long tloop = 0;

float sim() { 
  sim_cmd_time = (millis()*(PI/10000));
  sim_cmd = sin(sim_cmd_time); 
  return map(sim_cmd, -1, 1, 1000, 2000); 
}
