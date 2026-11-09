#pragma once
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <string>
#include <deque>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define A0 26
uint32_t micros(); uint32_t millis(); void delay(unsigned long); void delayMicroseconds(unsigned);
void pinMode(int,int); void digitalWrite(int,int); int digitalRead(int); int analogRead(int); void analogReadResolution(int);
struct SerialStub {
  std::deque<char> in; std::string out;
  void begin(long){} explicit operator bool(){return true;}
  int available(){return in.size();} int read(){char c=in.front(); in.pop_front(); return c;}
  void print(const char*s){out+=s;} void print(char c){out+=c;}
  void print(long v){out+=std::to_string(v);} void print(int v){out+=std::to_string(v);} void print(unsigned long v){out+=std::to_string(v);}
  void print(unsigned v){out+=std::to_string(v);} void print(int32_t v, int){out+=std::to_string(v);}
  void print(double v,int d){char b[32];snprintf(b,32,"%.*f",d,v);out+=b;} void print(float v,int d){print((double)v,d);}
  void println(){out+="\n";} template<class T> void println(T v){print(v);println();} void println(float v,int d){print(v,d);println();}
};
extern SerialStub Serial;
