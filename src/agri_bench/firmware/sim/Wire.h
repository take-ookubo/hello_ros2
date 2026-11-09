#pragma once
#include "arduino_stub.h"
struct TwoWire { void begin(){} void setClock(long){} void beginTransmission(int){} void write(int){} int endTransmission(){return 0;} };
extern TwoWire Wire;
