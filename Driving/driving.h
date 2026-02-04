#pragma once

// Motor B on H-bridge - RIGHT SIDE
#define ENA 4 //1
#define IN1 5 //2  
#define IN2 6 //3
#define rightEncA 8 //9   // yelliow wire - left motor
#define rightEncB 3 //10  // green wire - left motor

// Motor A on H-bridge - LEFT SIDE
#define ENB 16 //6
#define IN3 7 //4     
#define IN4 15 //5
#define leftEncA 17 //7   // green? wire - right motor
#define leftEncB 18 //8   // yellow? wire - right motor

#define pwmFreq 5000
#define pwmResolution 8

#define AUTO_SPEED    180   // slower to reduce skid at stop   // try changing to 250 if too slow
#define AUTO_RPM 100

#define ENC_PER_REV 700.0
#define ENC_PER_INCH 700.0/8.66

#define ENC_PER_BOTDEG 300.0/90.0   // num enc counts on each wheel for turning bot 90deg

void printEncoderCounts();

extern volatile long rightEncoderCount;
extern volatile long leftEncoderCount;

void IRAM_ATTR updateRightEncoder(); 
void IRAM_ATTR updateLeftEncoder(); 

float getCurrRPM(int motornum);

int right_velPIcontrol(float desired, float sensor);
int left_velPIcontrol(float desired, float sensor);

void setRightMotorSpeed(int u);
void setLeftMotorSpeed(int u);

void velPI_setRPMForTime(float left_targetRPM, float right_targetRPM, unsigned long duration_ms);
void driveFwdForTime(int ms);
void turnLeftForTime(int ms);
void turnRightForTime(int ms);
void driveBackForTime(int ms);

void turnBotDeg(float deg);
void driveBotInches(float inches);

void masher(int nummash);