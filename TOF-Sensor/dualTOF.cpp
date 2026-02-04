#include "Adafruit_VL53L0X.h"
#include "Wire.h"
#include "dualTOF.h"
#include "Arduino.h"


TwoWire tof_i2c_Wire = TwoWire(1);

// objects for the vl53l0x
Adafruit_VL53L0X frontTOF = Adafruit_VL53L0X();
Adafruit_VL53L0X sideTOF = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t frontDist;
VL53L0X_RangingMeasurementData_t sideDist;


bool init_sensors() {
  Serial.println("Initializing VL53L0X Sensors...");

  // 1. All sensors are held in reset (XSHUT is LOW)
  pinMode(FRONT_XSHUT, OUTPUT);
  pinMode(SIDE_XSHUT, OUTPUT);
  digitalWrite(FRONT_XSHUT, LOW);
  digitalWrite(SIDE_XSHUT, LOW);
  delay(100); // Give time for sensors to power down

  // --- Initialize Sensor A ---
  // 2. Bring Sensor A out of reset (XSHUT is HIGH)
  digitalWrite(FRONT_XSHUT, HIGH);
  delay(100);

  // 3. Initialize the sensor object with the dedicated I2C bus
  if (!frontTOF.begin(frontTOF_ADDRESS, false, &tof_i2c_Wire)) {
    Serial.println("Failed to detect & initialize VL53L0X Sensor Front.");
    return false;
  }
  
  // 4. Assign a new, unique address to Sensor A
  // The 'false' argument in lox_a.begin() prevents it from automatically
  // setting the address to 0x29. We now set it to 0x30.
  frontTOF.setAddress(frontTOF_ADDRESS);
  Serial.printf("Sensor A initialized and address set to 0x%02X\n", frontTOF_ADDRESS);


  // --- Initialize Sensor B ---
  // 5. Bring Sensor B out of reset (XSHUT is HIGH)
  digitalWrite(SIDE_XSHUT, HIGH);
  delay(100);

  // 6. Initialize Sensor B - it will be found at the *default* address (0x29)
  // because its XSHUT was just released and we are using the new TwoWire object.
  if (!sideTOF.begin(sideTOF_ADDRESS, false, &tof_i2c_Wire)) {
    Serial.println("Failed to detect & initialize VL53L0X Sensor B.");
    return false;
  }
  
  // 7. Assign a new, unique address to Sensor B
  sideTOF.setAddress(sideTOF_ADDRESS);
  Serial.printf("Sensor B initialized and address set to 0x%02X\n", sideTOF_ADDRESS);

  return true;
}

TOFReads readDualTOF() {
  TOFReads reads;
  
  frontTOF.rangingTest(&frontDist, false); // pass in 'true' to get debug data printout!
  sideTOF.rangingTest(&sideDist, false); // pass in 'true' to get debug data printout!

  if(frontDist.RangeStatus != 4) {     // if not out of range
    reads.front = frontDist.RangeMilliMeter;
    // Serial.print(frontDist.RangeMilliMeter);
  } else {
    reads.front = 5000;
    // Serial.print(F("Out of range"));
  }
  
  // print sensor two reading
  if(sideDist.RangeStatus != 4) {
    // Serial.print(sideDist.RangeMilliMeter);
    reads.side = sideDist.RangeMilliMeter;
  } else {
    reads.side = 5000;
    // Serial.print(F("Out of range"));
  }
  
  return reads;
}