//---------- CONFIG ----------// 

#define I_WANT_TO_FLY   false // Simulated servo movement to test the servo movement :)) 
#define TEST_DIR_RC     false // Use channels 0 on the radio to test the direction of the autopilot and the servos, I_WANT_TO_FLY should be set true too. 
#define TEST_SEP_SERVO  false // Will simulate a separation 10 seconds after start up 
#define BUZZER_TURN     false // Buzzer sounds as function of the turn command 
#define BUZZER_SWEEP    false // Buzzer turn on steroïds, should be easier to understand his tricky language ^^
#define NO_INIT         false // Skip init, for testing only purposes 
#define NAV_WAYPOINT    true  // Doing the waypoint sequence before reaching home? 
#define NAV_HOME        true  // Should it go home after the waypoint sequence? 
#define SD_WRITING      true  // Should it write on the SD card or not? 
#define LOW_RATE        false // Dataloging at low HZ (if true) instead of 20Hz, for balloon flight 
#define CONFIG_FILE_SV  false // Will also save the config file with the name of the file to debug if the config was wrong 
#define LED_MODEL       0     // Big led = 1, small led = 0. 
#define CAM_CYCLE       true  // If true, camera will only be powered on for 20s every 10min, used to save battery on long HAB flight 
#define SAFE_TRIGGER    false // For HAB flight, will use a safer, but slower methode to detect apogee and transision to descent mode 
#define COG_BRAKE       false // Will reduce command if CoG is turning faster than a threshold
#define GPS_FREQ        5     // Hz
#define SERVO_RATE      50    // Hz 
#define TLM_MONITOR     true  // Show telemetry on serial monitor 
#define DEBUG           false // Will show all the steps on the screen
#define GPS_PORT Serial7
#define TLM_PORT Serial5 
#define RX_PORT  Serial1
#define TIME_OUT 300

//---------- FLIGHT SETTINGS ----------// 

#define RC_MODE         1     // only roll (0), pitch and roll mixed (1), pitch and roll separated (2)
#define CONTROL_MODE    1     // neutral position at the center (0) or with "hands up" (1) 
#define LINEAR_MODE     0     // command is linear (0), or linear but with a large deadband set using servo_start etc (1)
#define DROP            true  // R2Home's version, drop or motorised

#define DEP_MODE        1     // If 0, the ALTITUDE will be used if 1, the TIMER will be used, to trigger deployment once in descent mode
#define DESCENT_TIMER   4000  
#define OPENING_TIMER   3000
#define SPIRAL_RECOVER  5000
#define DEP_ALT         300   // m above ground altitude 
#define SEP_ALT         10    // m above sea level
#define VUP             1     // m/s 
#define VDOWN           -1    // m/s

// PID for navigation, better to not touch them and touch the servo max command settings instead 
#define NKP   1
#define NKI   0.05
#define NKD   0.1

#define BCRITICAL 3.4
#define BLOW      3.5
#define NO_BATT   4.0 

#define AUTO_GAIN_WEIGHT    true  // Servo max command will be set automatically at initialization based on given payload weight 
#define AUTO_GAIN_PRESSURE  true  // Servo max command will be set automatically during the flight based on the ratio between ground pressure and current pressure 

#define SYSTEM_WEIGHT  500 // System weight in gram
#define PAYLOAD_WEIGHT 500 // Payload weight in gram 

// If you choose to not use any of the two auto_gain functions, you can set the max commands with these two lines: 
#define SERVO_MAX_M_DEF 2000 // m for map
#define SERVO_MAX_C_DEF 2000 // c for constrain 

#define TRIG 20
#define LEFT_OFFSET   100
#define RIGHT_OFFSET  100 

#define HOME_WAYPOINT_THRESHOLD 10 // Distance to waypoint before going to the next waypoint

#define BARO_VS_AVG 1
#define BARO_VS_SAMPLE 50

#define BARO_AL_AVG 50
#define BARO_PS_AVG 1

#define GPS_VS_AVG  1

#define GPS_SAFE_AVG  10
#define BARO_SAFE_AVG 5

#define PRE_PE_AVG 50
