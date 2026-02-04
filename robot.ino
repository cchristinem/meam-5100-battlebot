#include "Adafruit_VL53L0X.h"
#include "Wire.h"
#include "dualTOF.h"
#include "driving.h"
#include "auto_routines.h"



int health_points = 0;

/* ----- Vive -----*/ 
#include "vive510.h"
#define SIGNALPIN1 38 //40 // pin receiving signal from Vive circuit
Vive510 vive1(SIGNALPIN1);
#define FREQ 1 // in Hz
static uint16_t x,y;

uint32_t med3filt(uint32_t a, uint32_t b, uint32_t c) {
  uint32_t middle;
  if ((a <= b) && (a <= c))
    middle = (b <= c) ? b : c;  
  else if ((b <= a) && (b <= c))
    middle = (a <= c) ? a : c;
  else    middle = (a <= b) ? a : b;
  return middle;
}

void updateViveCoords(){
  if (vive1.status() == VIVE_RECEIVING) {
    static uint16_t x0, y0, oldx1, oldx2, oldy1, oldy2;
    oldx2 = oldx1; oldy2 = oldy1;
    oldx1 = x0;     oldy1 = y0;
    
    x0 = vive1.xCoord();
    y0 = vive1.yCoord();
    x = med3filt(x0, oldx1, oldx2);
    y = med3filt(y0, oldy1, oldy2);
    // digitalWrite(LED_BUILTIN,HIGH);
    if (x > 8000 || y > 8000 || x< 1000 || y < 1000) {
      x=0; y=0;
      // digitalWrite(LED_BUILTIN,LOW);
    }
    // Serial.print(x); Serial.print("    "); Serial.println(y);
  }
  else {
    // digitalWrite(LED_BUILTIN,LOW);
    x=0;
    y=0; 
    vive1.sync(5); 
  }
}


/* ----- TopHat ----- */
#define TH_I2C_SLAVE_ADDR 0x28
#define TH_SDA_PIN 48 //34
#define TH_SCL_PIN 47 //33

TwoWire th_i2c_Wire = TwoWire(0);

void send_I2C_byte(uint8_t data) {
  // Send data to slave
  th_i2c_Wire.beginTransmission(TH_I2C_SLAVE_ADDR);
  th_i2c_Wire.write(data);  // Send some test data
  uint8_t error = th_i2c_Wire.endTransmission();

  if (error == 0) {
    Serial.println("Data sent successfully");
    // rgbLedWrite(2, 0, 20, 0);  // green
  } else {
    Serial.printf("Error sending data: %d\n", error);
    // rgbLedWrite(2, 20, 0, 0);  // red
  }
}

uint8_t receive_I2C_byte() {  // data should have space declared from caller
  // Request data from slave
  uint8_t bytesReceived = th_i2c_Wire.requestFrom(TH_I2C_SLAVE_ADDR, 1);
  uint8_t byteIn = 0;

  if (bytesReceived > 0) {
    Serial.print("Received from slave: ");
    while (th_i2c_Wire.available()) {
      byteIn = th_i2c_Wire.read();
      Serial.printf("0x%02X ", byteIn);
    }
    Serial.println();
  } else return 0;
  return byteIn;  // return number of bytes read
}


/* ----- Servo ----- */
const int servoPin = 1; 
const int channel = 0; 
const int minDuty = 25;   
const int maxDuty = 128;  
bool servoisactive = false;

void sweepServo(){
  if (servoisactive){
    for (int angle = 0; angle <= 180; angle += 1) {
      setServoAngle(angle);
      delay(10); // Small delay to slow down the sweep
    }
    
    for (int angle = 180; angle >= 0; angle -= 1) {
      setServoAngle(angle);
      delay(10);
    }
  }
  else{
    setServoAngle(0);
  }
}

void setServoAngle(int angle) {
  if (servoisactive){
    int dutyCycle = map(angle, 0, 180, minDuty, maxDuty);
    ledcWrite(servoPin, dutyCycle);
    Serial.printf("Angle: %d -> Duty: %d\n", angle, dutyCycle);
  }
  else{
    ledcWrite(servoPin, 25);
  }
  
}


/* ----- AUTONOMOUS ROUTINES ----- */
bool wallFollowingMode = false; 
bool autonomousMode = false;

// dynamic vars to store vive targets
int nexus_targX = 0;
int nexus_targY = 0;

// nexus coords in vive
#define RED_TARGET_X  4650
#define RED_TARGET_Y  6000

#define BLUE_TARGET_X 4250
#define BLUE_TARGET_Y 2250

#define POS_TOLERANCE 50  


// NOTE: wall following with right turns around track
void wallFollow() {

    const int TOTAL_TIME = 60000; // wall following ends after 60s
    const int SENSOR_WALL_SIDE = 150; // this value means that it is too close and needs to turn
    const int SENSOR_WALL_FRONT = 120; // this value means that it is too close and needs to turn
    const int LAST_TURN = 8;
    const int RAMP_TURN = 5;
    const int RAMP_UP_TIME = 2000; // fix this

    int WF_SPEED = 180;

    double prev_side = SENSOR_WALL_SIDE; // init to closest allowable dist
    unsigned long start = millis();
    unsigned long ramp_start = start + 15000; // 15 seconds from start time we go up ramp for example
    int turn_count = 0; 

    while ((millis() - start) < TOTAL_TIME && wallFollowingMode) { 
      /* TOF reading */
      TOFReads reads = readDualTOF();
      /* PRINT ALL DATA */ 
      Serial.print("VIVE_X: "); Serial.print(x); Serial.print(" VIVE_Y: "); Serial.print(y); Serial.print("\t TOF_FRONT: "); Serial.print(reads.front); Serial.print("\t TOF_SIDE: "); Serial.print(reads.side); Serial.print("\t "); printEncoderCounts();
    
      

      // if front sensor detects wall
      if (reads.front <= SENSOR_WALL_FRONT) { 
        // turn right
        setLeftMotorSpeed(-WF_SPEED);
        setRightMotorSpeed(-WF_SPEED);
        delay(150);
        //turnBotDeg(60);
        setRightMotorSpeed(0);
        setLeftMotorSpeed(0);
        delay(10);
  
        setLeftMotorSpeed(WF_SPEED);
        setRightMotorSpeed(-WF_SPEED);
        delay(400);

        setRightMotorSpeed(0);
        setLeftMotorSpeed(0);
        delay(10);

      
        turn_count = turn_count + 1;

        Serial.println("\nWALL, turn RIGHT\n");
      }
      
      // if side sensor too close and getting closer
      else if (reads.side <= SENSOR_WALL_SIDE) { //  && prev_side > reads.side
        // drive with bias to the right
        setLeftMotorSpeed(WF_SPEED);
        setRightMotorSpeed(0.75*WF_SPEED);
        delay(250);
        setLeftMotorSpeed(0);
        setRightMotorSpeed(0);
        
        Serial.println("\nTOO CLOSE, bias RIGHT\n");
      } 
      
      // if side sensor too far and getting farther
      else if (reads.side >= SENSOR_WALL_SIDE) { //  && prev_side < reads.side
        // drive with bias to the left
        setLeftMotorSpeed(0.75*WF_SPEED);
        setRightMotorSpeed(WF_SPEED);
        delay(250);
        setLeftMotorSpeed(0);
        setRightMotorSpeed(0);

        Serial.println("\nTOO FAR, bias LEFT\n");
      } 

      // otherwise
      else {
        // drive straight
        setLeftMotorSpeed(WF_SPEED);
        setRightMotorSpeed(WF_SPEED);
        delay(300);

        Serial.println("\nNO interference, drive STRAIGHT\n");
      }

      // short stop
      setLeftMotorSpeed(0);
      setRightMotorSpeed(0);
      prev_side = reads.side;
      delay(100); // decrease later

      Serial.println("\n< quick reset >\n");
    }
}
    

/* ----- Vive Navigation ----- */
bool viveNavigateMode = false; 
long x_in;
long y_in;

// NOTE: wall following with right turns around track
void viveNavigate(long x_vive, long y_vive) {
  // always start from bottom right corner of field!
  if (viveNavigateMode) { 
    Serial.print("VIVE: Target X: "); Serial.println(x_vive); Serial.println("Target Y: "); Serial.println(y_vive);
    updateViveCoords();
    mapViveCoords(x_vive, y_vive);
    Serial.print("FIELD: Target X: "); Serial.println(x_in); Serial.println("Target Y: "); Serial.println(y_in);
    followPath(x_in, y_in);
    setLeftMotorSpeed(0);
    setRightMotorSpeed(0);
    viveNavigateMode = false;
  }
  return;
}

void mapViveCoords(long x_vive, long y_vive) {
  // option 1, linear regression from data points in excel
  x_in = 0.023825 * x_vive + (-68.3019);
  y_in = 0.028477 * y_vive + (-49.2518);

  Serial.print("Mapped X (in): ");
  Serial.println(x_in);
  Serial.print("Mapped Y (in): ");
  Serial.println(y_in);
}

void followPath(long x_in, long y_in) {
  // if interferes with box, only happens if on upper of box
  driveBotInches(144 - y_in); 
  turnBotDeg(90);
  driveBotInches(60 - x_in);
}


/* ----- Web and Handlers -----  */
#include "body.h"
#include "html510.h"
HTML510Server h(80);
const char* ssid     = "meow";
const char* password = "";

static int des_leftRPM = 0;
static int des_rightRPM = 0;
static uint8_t release_i2c_byte = 0x00; // Default to 0x00 (STOP command)

void handleRoot() {
  h.sendhtml(body);
}

// Forward: both motors forward
void handleForward() {
  // des_leftRPM = 100; // 100;
  // des_rightRPM = 100; // 100;
  setLeftMotorSpeed(AUTO_SPEED); // LATER ADD PID!!!!!
  setRightMotorSpeed(AUTO_SPEED);
  h.sendplain("Moving Forward");
}

// Backward: both motors backward
void handleBackward() {
  setLeftMotorSpeed(-AUTO_SPEED); // LATER ADD PID!!!!!
  setRightMotorSpeed(-AUTO_SPEED);
  h.sendplain("Moving Backward");
}

// Left: left motor backward, right motor forward
void handleLeft() {
  setLeftMotorSpeed(-AUTO_SPEED);
  setRightMotorSpeed(AUTO_SPEED);
  //turnBotDeg(-90.0);
  h.sendplain("Turning Left");
}

// Right: right motor backward, left motor forward
void handleRight() {
  setLeftMotorSpeed(AUTO_SPEED);
  setRightMotorSpeed(-AUTO_SPEED);
  //turnBotDeg(90.0);
  h.sendplain("Turning Right");
}

// Stop: halt both motors
void handleStop() {
  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);
  h.sendplain("Stopped");
}

// New handler for button release
void handleReleaseStop() {
  // Set the I2C byte to be sent in the main loop
  release_i2c_byte = 0x01; // For example, set to 0x01 for a specific 'stop' command
  health_points = health_points-1;
  // Also stop the motors
  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);

  if (health_points == 0) {
    delay(15000);
    health_points = 100;
  }
  h.sendplain("Release Stop Received");
}

void handleServo(){
  Serial.println("SERVOD UP ON A FIRDAYA");
  if(!servoisactive){
    servoisactive = true;
    Serial.println("set bool to true");
  }
  else{
    servoisactive = false;
    Serial.println("set bool to false");
  }
  
  setServoAngle(120);
}

// Max forward for ramp
void handleRampForward() {
  // des_leftRPM = 100; // 100;
  // des_rightRPM = 100; // 100;
  setLeftMotorSpeed(225); // LATER ADD PID!!!!!
  setRightMotorSpeed(225);
  h.sendplain("Ramp Forward");
}

// autonomous high blue tower attack
void handleHitHighBlueTower() {
  autonomousMode = true;
  hitHighBlueTower();
}

// autonomous high blue tower attack
void handleHitLowBlueTower() {
  autonomousMode = true;
  hitLowBlueTower();
}

// autonomous blue nexus attack
void handleBlueNexus() {
  autonomousMode = true;
  nexus_targX = BLUE_TARGET_X;
  nexus_targY = BLUE_TARGET_Y;

  hitBlueNexus();
  autonomousMode = false;
}

// abort autonomous mode 
void handleAbort() {
  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);
  autonomousMode = false;
  wallFollowingMode = false;
}

void handleWallFollow() {
  wallFollowingMode = true;
  wallFollow();
  h.sendplain("Wall following started");
}

void handleViveNavigate() {
  viveNavigateMode = true;
  int coords = h.getVal();
  long xtarget = coords / 10000;
  long ytarget = coords % 10000;
  h.sendplain("Vive navigation started");
  viveNavigate(xtarget, ytarget);
}


/* ----- SETUP ----- */
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("hey!");

  /* Web Setup */
  delay(2000);
  WiFi.softAP(ssid, password);
  Serial.println(ssid);
  Serial.print("AP IP address: http://");  
  Serial.println(WiFi.softAPIP());
  h.begin();
  
  // attach auton stuff
  h.attachHandler("/", handleRoot);
  // attach other stuff
  h.attachHandler("/forward", handleForward);
  h.attachHandler("/backward", handleBackward);
  h.attachHandler("/right", handleRight); // swapped, error
  h.attachHandler("/left", handleLeft); // swapped, error
  h.attachHandler("/stop", handleStop);
  h.attachHandler("/release_stop", handleReleaseStop);
  h.attachHandler("/blue_high_tower", handleHitHighBlueTower);
  h.attachHandler("/blue_low_tower", handleHitLowBlueTower); 
  h.attachHandler("/blue_nexus", handleBlueNexus);
  h.attachHandler("/abort", handleAbort); 
  h.attachHandler("/wall", handleWallFollow); 
  h.attachHandler("/servo", handleServo);
  h.attachHandler("/ramp", handleRampForward);
  h.attachHandler("/", handleRoot);
  h.attachHandler("/nav", handleViveNavigate);

  /* Motor Setup */
  // left motor setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(leftEncA, INPUT_PULLUP);
  pinMode(leftEncB, INPUT_PULLUP);
  ledcAttach(ENA, pwmFreq, pwmResolution);
  attachInterrupt(digitalPinToInterrupt(leftEncA), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncB), updateLeftEncoder, CHANGE);
  // right motor setup
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(rightEncA, INPUT_PULLUP);
  pinMode(rightEncB, INPUT_PULLUP);
  ledcAttach(ENB, pwmFreq, pwmResolution);
  attachInterrupt(digitalPinToInterrupt(rightEncA), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncB), updateRightEncoder, CHANGE);
  // set initial motor states (stopped)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
  //servo setup
  ledcAttach(servoPin, 50, 10); //pin, freq, res

  delay(50);

  /* Vive Setup */ 
  pinMode(LED_BUILTIN,OUTPUT);
  vive1.begin();
  Serial.println("Vive trackers= started");


  /* tophat i2c */
  th_i2c_Wire.begin(TH_SDA_PIN, TH_SCL_PIN, 40000);
  
  /* TOF and i2c setup */
  tof_i2c_Wire.begin(TOF_SDA_PIN, TOF_SCL_PIN);
  // setup TOF xshut pins, disable, configure TOFs
  if (!init_sensors()){
    while (1) {
      Serial.println("VL53L0X initialization failed. Halting.");
      delay(1000);
    }
  }

  delay(50);
}


/* ----- MAIN LOOP ----- */
void loop() {
  /* Web and i2c health */
  h.serve();
  //send_I2C_byte(0x55);
  //tophat - Send I2C only on button release command 
  if (release_i2c_byte != 0x00) {
    Serial.print("Sending release I2C byte: 0x"); Serial.println(release_i2c_byte, HEX);
    send_I2C_byte(release_i2c_byte);
    receive_I2C_byte();       // loop back
    release_i2c_byte = 0x00;  // Reset the flag
  }
    
  delay(10);
}