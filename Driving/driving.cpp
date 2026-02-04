#include "esp32-hal.h"
#include "esp32-hal-ledc.h"
#include "esp32-hal-gpio.h"
#include "HardwareSerial.h"
#include "Arduino.h"
#include "driving.h"

// Timing for print updates
unsigned long lastPrintTime = 0;
const int printInterval = 500;  // Print every 500ms

// Motor adjustment factor, if one motor weaker than the other
const float MAF = 0.982; // decrease right motor w adjustment factor

// encoder variables
extern volatile long rightEncoderCount = 0;
extern volatile long leftEncoderCount = 0;

void printEncoderCounts(){
  Serial.print("RightEnc: "); Serial.print(rightEncoderCount); Serial.print("\t"); Serial.print("LeftEnc: "); Serial.println(leftEncoderCount);
}




// Encoder ISR for each motor
volatile int rightLastEncoded = 0;
volatile int leftLastEncoded = 0;
void IRAM_ATTR updateRightEncoder() {
  int MSB = digitalRead(rightEncA);
  int LSB = digitalRead(rightEncB);
  
  int encoded = (MSB << 1) | LSB;
  int sum = (rightLastEncoded << 2) | encoded;
  
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) rightEncoderCount--;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) rightEncoderCount++;
  
  rightLastEncoded = encoded;
}
void IRAM_ATTR updateLeftEncoder() {
  int MSB = digitalRead(leftEncA);
  int LSB = digitalRead(leftEncB);
  
  int encoded = (MSB << 1) | LSB;
  int sum = (leftLastEncoded << 2) | encoded;
  
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) leftEncoderCount--;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) leftEncoderCount++;
  
  leftLastEncoded = encoded;
}




long lastRightCount = 0;
long lastLeftCount  = 0;

unsigned long lastRightTime = 0;
unsigned long lastLeftTime  = 0;

float getCurrRPM(int motornum){
    unsigned long now = millis();
    unsigned long dt;
    long curr, delta;
    long* lastCount;
    unsigned long* lastTime;

    if (motornum == 1){
        curr = rightEncoderCount;
        lastCount = &lastRightCount;
        lastTime  = &lastRightTime;
    } else {
        curr = leftEncoderCount;
        lastCount = &lastLeftCount;
        lastTime  = &lastLeftTime;
    }

    dt = now - *lastTime;
    if (dt < 50) return 0;         // too soon, avoid noise

    delta = curr - *lastCount;

    *lastCount = curr;
    *lastTime  = now;

    if (delta == 0) return 0;      // no movement

    // RPM = delta counts / (counts per rev) / (dt sec) * 60 sec/min
    float rpm = (float)delta / ENC_PER_REV * (60000.0 / dt);

    return rpm;
}




// consts for Velocity PI support
#define r_KI 0.05
#define r_KP 0.75 // 0.33 //4.0
#define r_KD 0.002

#define l_KI 0.049 
#define l_KP 0.75 
#define l_KD 0.002

// global states for PI control
float r_sumErr = 0;
float l_sumErr = 0;

float right_oldsensor = 0;
float left_oldsensor = 0;




// --- RPM STATE RESET ---
// Fix: use the actual variables used by getCurrRPM(): 
// lastRightCount, lastLeftCount, lastRightTime, lastLeftTime
void resetRPM() {
    unsigned long now = millis();
    lastRightTime = now;
    lastLeftTime  = now;

    // sync last counts to current encoder readings so the next delta is measured from here
    lastRightCount = rightEncoderCount;
    lastLeftCount  = leftEncoderCount;
}

// --- FULL CONTROL RESET BEFORE NEW MOVEMENT ---
void resetVelocityControllers() {
    r_sumErr = 0;
    l_sumErr = 0;
    right_oldsensor = 0;
    left_oldsensor = 0;
    resetRPM();
}




int right_velPIcontrol(float desired, float sensor) {
    int u;

    if (desired == 0) {
        r_sumErr = 0;
    } else {
        r_sumErr += (desired - sensor);
    }

    u = r_KP * (desired - sensor)
      + r_KI * r_sumErr
      + r_KD * (right_oldsensor - sensor);

    u = constrain(u, -255, 255);

    right_oldsensor = sensor;
    return u;
}

int left_velPIcontrol(float desired, float sensor) {
    int u;

    if (desired == 0) {
        l_sumErr = 0;
    } else {
        l_sumErr += (desired - sensor);
    }

    u = l_KP * (desired - sensor)
      + l_KI * l_sumErr
      + l_KD * (left_oldsensor - sensor);

    u = constrain(u, -255, 255);

    left_oldsensor = sensor;
    return u;
}




void setRightMotorSpeed(int u){
  if (u==0){
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, LOW);
    ledcWrite(ENB, 0);
  }
  else {
    if (u>0){
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      ledcWrite(ENB, u);
    }
    else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      ledcWrite(ENB, -u);
    }
  }
}

void setLeftMotorSpeed(int u){
  if (u==0){
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, LOW);
    ledcWrite(ENA, 0);
  }
  else {
    if (u>0){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      ledcWrite(ENA, u);
    }
    else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      ledcWrite(ENA, -u);
    }
  }
}




void velPI_setRPMForTime(float left_targetRPM, float right_targetRPM, unsigned long duration_ms) {
    unsigned long start = millis();
    const unsigned long dt_control = 50;   // control loop update rate (~20 Hz)
    unsigned long lastLoop = millis();

    while (millis() - start < duration_ms) {

        if (millis() - lastLoop >= dt_control) {
            lastLoop = millis();

            // --- Measure current velocities ---
            float right_rpm = getCurrRPM(1);  
            float left_rpm  = getCurrRPM(2);  

            // --- Compute control outputs ---
            int u_right = right_velPIcontrol(right_targetRPM, right_rpm);
            int u_left  = left_velPIcontrol(left_targetRPM,  left_rpm);

            // --- Drive motors ---
            setRightMotorSpeed(u_right);
            setLeftMotorSpeed(u_left);
        }

        // IMPORTANT: let ISR fire
        delay(1);
    }

    // --- Stop motors at the end ---
    setRightMotorSpeed(0);
    setLeftMotorSpeed(0);
}




void driveFwdForTime(int ms){
  setLeftMotorSpeed(AUTO_SPEED);
  setRightMotorSpeed(AUTO_SPEED);
  delay(ms);
  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);
}

void turnLeftForTime(int ms){
  setLeftMotorSpeed(-AUTO_SPEED);
  setRightMotorSpeed(AUTO_SPEED);
  delay(ms);
  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);
}
void turnRightForTime(int ms){
  setLeftMotorSpeed(AUTO_SPEED);
  setRightMotorSpeed(-AUTO_SPEED);
  delay(ms);
  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);
}
void driveBackForTime(int ms){
  setLeftMotorSpeed(-AUTO_SPEED);
  setRightMotorSpeed(-AUTO_SPEED);
  delay(ms);
  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);
}



void driveBotInches(float inches) {

    // Reset for new movement
    rightEncoderCount = 0;
    leftEncoderCount  = 0;
    resetVelocityControllers();

    // Target counts with proper sign
    long tar = (long)(inches * ENC_PER_INCH);

    while (true) {

      long r = rightEncoderCount;
      long l = leftEncoderCount;

      long errR = tar - r;
      long errL = tar - l;

      // Stop when both wheels reach or pass target
      if ((tar > 0 && errR <= 0 && errL <= 0) ||
          (tar < 0 && errR >= 0 && errL >= 0)) {
          break;
      }

      // Simple proportional pre-command (desired wheel RPM)
      // --- Speed profile ---
      // Slower overall speed + smooth decel near target
      float distErr = max(abs(errR), abs(errL));
      float maxRPM = 35;          // slower top speed
      float minRPM = 12;          // crawl speed near stop
      float slowZone = 0.25 * abs(tar);  // start slowing at last 25%

      float rampRPM;
      if (distErr < slowZone) {
          rampRPM = minRPM + (maxRPM - minRPM) * (distErr / slowZone);
      } else {
          rampRPM = maxRPM;
      }

      // --- Heading correction (distance equalization) ---
      long diff = r - l;              // + means right wheel ahead
      float kHeading = 0.02;         // steering gain (tune 0.01–0.03)

      float steer = kHeading * diff;

      float baseR = rampRPM - steer;
      float baseL = rampRPM + steer;

      // Apply direction of travel
      if (tar > 0) {
          // forward
          baseR =  baseR;
          baseL =  baseL;
      } else {
          // backward
          baseR = -baseR;
          baseL = -baseL;
      }

      // PI velocity control
      int uR = right_velPIcontrol(baseR, getCurrRPM(1));
      int uL = left_velPIcontrol(baseL,  getCurrRPM(2));

      setRightMotorSpeed(uR);
      setLeftMotorSpeed(uL);

      delay(20);
    }

  // --- Gentle stop to avoid veer ---
  velPI_setRPMForTime(10, 10, 60);
  // velPI_setRPMForTime(8, 8, 60);
  setRightMotorSpeed(0);
  setLeftMotorSpeed(0);
}



void turnBotDeg(float tardeg) {

    rightEncoderCount = 0;
    leftEncoderCount = 0;

    resetVelocityControllers();   // <<< FIX #1: fresh PI & RPM

    float tar = ENC_PER_BOTDEG * abs(tardeg);

    while (true) {

        int r = abs(rightEncoderCount);
        int l = abs(leftEncoderCount);

        int errR = tar - r;
        int errL = tar - l;

        if (errR <= 0 && errL <= 0) break;

        int uR = constrain(errR * 0.2, 15, 25);
        int uL = constrain(errL * 0.2, 15, 25);
        Serial.print(uL); Serial.print("\t"); Serial.println(uR);

        if (tardeg > 0) {
            uR = right_velPIcontrol(-uR, getCurrRPM(1));
            uL = left_velPIcontrol(uL, getCurrRPM(2));
            setRightMotorSpeed(uR);
            setLeftMotorSpeed(uL);
        } else {
            uR = right_velPIcontrol(uR, getCurrRPM(1));
            uL = left_velPIcontrol(-uL, getCurrRPM(2));
            setRightMotorSpeed(uR*1.025);
            setLeftMotorSpeed(uL*1.025);
        }

        delay(20);
    }

    setRightMotorSpeed(0);
    setLeftMotorSpeed(0);
}

void masher(int nummash){
  for (int i=0; i<nummash * 2; i++){
    // driveBotInches(5);
    // delay(100);
    // driveBotInches(-2);
    // delay(100);
    driveFwdForTime(500);
    delay(100);
    driveBackForTime(250);
  }
}





