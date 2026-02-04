#include "Adafruit_VL53L0X.h"
#include "Wire.h"
#include "dualTOF.h"
#include "driving.h"
#include "Arduino.h"

void hitBlueNexus(){
  // hits blue nexus 4 times
  driveBotInches(136);  delay(1000);
  driveBotInches(-4); delay(500);
  turnBotDeg(-90); delay(500);
  driveBotInches(12);
  delay(250);
  masher(4);
  return;
}

void hitHighBlueTower(){
  // drive up the ramp and hit blue button
  // driveBotInche(11); delay(2500);
  // turnBotDeg(-90.0); delay(2500);
  driveBotInches(83); 
  Serial.println("up ramp"); 
  delay(2500); 
  
  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);
  delay(100);

  turnBotDeg(60.0); 
  Serial.println("face button"); 
  delay(2500); 

  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);
  delay(100);

  driveBotInches(20); 
  delay(3000);

  Serial.println("*****BLUE HIGH TOWER YASSED*****");
}


void hitLowBlueTower(){
  driveBotInches(80); 
  delay(2500);
  turnBotDeg(-90.0); delay(2500);
  driveBotInches(7); delay(2500);
  turnBotDeg(-90.0); delay(2500);
  driveBotInches(20); 
}

void wallFollowCW(){
  driveBotInches(128); delay(2500);
  turnBotDeg(90.0); delay(2500);  // corner
  driveBotInches(6); delay(2500);

  turnBotDeg(90.0); delay(2500);  // around blue nexus
  driveBotInches(6); delay(2500);
  turnBotDeg(-90.0); delay(2500);
  driveBotInches(11); delay(2500);
  turnBotDeg(-90.0); delay(2500);
  driveBotInches(6); delay(2500);
  turnBotDeg(90.0); delay(2500);

  driveBotInches(18); delay(2500);  // ramp
  turnBotDeg(90.0); delay(2500);
  driveBotInches(128); delay(2500);
  turnBotDeg(90.0); delay(2500);
  driveBotInches(18); delay(2500);

  turnBotDeg(90.0); delay(2500);  // around red nexus
  driveBotInches(6); delay(2500);
  turnBotDeg(-90.0); delay(2500);
  driveBotInches(11); delay(2500);
  turnBotDeg(-90.0); delay(2500);
  driveBotInches(6); delay(2500);
  turnBotDeg(90.0); delay(2500);

  driveBotInches(6); delay(2500);

}