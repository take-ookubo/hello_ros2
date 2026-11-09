#pragma once
#include "Wire.h"
#define NAU7802_GAIN_128 7
#define NAU7802_SPS_80 3
#define NAU7802_SPS_320 7
struct NAU7802 { bool begin(TwoWire&){return true;} void setGain(int){} void setSampleRate(int){} bool calibrateAFE(){return true;}
  bool available(); int32_t getReading(){return 1000;} };
