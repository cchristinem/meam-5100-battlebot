#pragma once

#define TOF_SCL_PIN 19 //20
#define TOF_SDA_PIN 20 //21
// address we will assign if dual sensor is present
#define frontTOF_ADDRESS 0x30
#define sideTOF_ADDRESS 0x31

// set the pins to shutdown
#define FRONT_XSHUT 9 //12
#define SIDE_XSHUT 46 //11

extern TwoWire tof_i2c_Wire;

struct TOFReads {
  int front;
  int side;
};

bool init_sensors();
TOFReads readDualTOF();