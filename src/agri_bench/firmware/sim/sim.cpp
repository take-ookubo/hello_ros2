// Host simulation of the stage part of nau7802_logger.ino: stub Arduino
// functions, a simulated clock and a carriage with a home switch, a far
// switch and a motor supply. Runs the sketch's loop()/loop1() and checks
// the replies and the motion. Build and run with ./run.sh (needs g++).
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
#include "arduino_stub.h"
#include <cstring>
#include <cctype>
#include <cstdlib>
#include <vector>
#include <cassert>
static int failures = 0;
static void check(bool ok, const char* what){ printf("%s %s\n", ok?"PASS":"FAIL", what); if(!ok) failures++; }
TwoWire Wire; SerialStub Serial;
static uint64_t now_us = 0;
uint32_t micros(){ return (uint32_t)now_us; } uint32_t millis(){ return now_us/1000; }
void delay(unsigned long){} void delayMicroseconds(unsigned u){ now_us += u; }
// carriage: real position in steps; home switch pressed when real <= HOME_AT
static int64_t real = 0; static int64_t HOME_AT = -1600*3; static int64_t MAX_AT = 1600*102 /*mm*/; // relative to start
static int dirpin = 0; static int steppin = 0; static bool power = true; static bool min_broken = false;
static std::vector<std::pair<uint64_t,int64_t>> trace;
int pinval[40];
void pinMode(int,int){}
void digitalWrite(int p,int v){ if(p==3) dirpin=v; if(p==2){ if(v==HIGH && steppin==LOW && power){ real += dirpin?1:-1; trace.push_back({now_us,real}); } steppin=v; } }
int digitalRead(int p){ if(p==7) return (min_broken || real<=HOME_AT)?HIGH:LOW; if(p==8) return real>=MAX_AT+HOME_AT?HIGH:LOW; if(p==9) return power?HIGH:LOW; return LOW; }
int analogRead(int){return 0;} void analogReadResolution(int){}
bool NAU7802::available(){ static uint64_t last=0; if(now_us-last>=3125){last=now_us;return true;} return false; }
#include "../nau7802_logger/nau7802_logger.ino"
void send(const char*s){ while(*s) Serial.in.push_back(*s++); Serial.in.push_back('\n'); }
std::string run_until(const char* word, double max_s){
  uint64_t end = now_us + (uint64_t)(max_s*1e6);
  size_t start = Serial.out.size();
  while(now_us < end){ loop(); loop1(); now_us += 1;
    std::string tail = Serial.out.substr(start);
    // drop data lines
    size_t pos=0; std::string msgs;
    while(pos<tail.size()){ size_t e=tail.find('\n',pos); if(e==std::string::npos) break; std::string l=tail.substr(pos,e-pos); if(l.rfind("# ",0)==0) msgs+=l+"\n"; pos=e+1; }
    if(word && msgs.find(word)!=std::string::npos) return msgs;
    if(!word && now_us+1>=end) return msgs; }
  std::string tail = Serial.out.substr(start), msgs; size_t pos=0;
  while(pos<tail.size()){ size_t e=tail.find('\n',pos); if(e==std::string::npos) break; std::string l=tail.substr(pos,e-pos); if(l.rfind("# ",0)==0) msgs+=l+"\n"; pos=e+1; }
  return msgs + "(timeout)\n";
}
int main(){
  setup(); setup1();
  send("MOVE 5 5 @1");
  check(run_until("error",0.1).find("error not homed id=1")!=std::string::npos, "MOVE before HOME is refused");
  send("HOME @2");
  check(run_until("homed",30).find("homed id=2")!=std::string::npos, "HOME finds the switch");
  check(real == HOME_AT + 1 && stage_pos == 0, "position 0 is where the switch releases");
  int64_t real0 = real;
  trace.clear(); send("GOTO 50 150 @3");
  check(run_until("done",5).find("done pos=50.000 id=3")!=std::string::npos, "GOTO 50 at 150 mm/s ends");
  check(real - real0 == 50*160 && stage_pos == 50*160, "carriage moved exactly 50 mm");
  { size_t n=trace.size(); auto a=trace[n/2-100], b=trace[n/2+100];
    double v=(b.second-a.second)/160.0/((b.first-a.first)*1e-6);
    printf("     top speed %.2f mm/s\n", v); check(fabs(v-150.0) < 0.5, "top speed within 0.5 mm/s of 150");
    // distance until the speed first reaches 99 % of the top speed
    // (speed over 20 steps: one interval alone is only a few whole microseconds)
    double ramp = 0; for (size_t k = 20; k < trace.size(); k++) {
      double vk = 20.0/160.0 / ((trace[k].first - trace[k-20].first)*1e-6);
      if (vk >= 0.99*150.0) { ramp = (trace[k].second - trace.front().second)/160.0; break; } }
    printf("     distance to reach 148.5 mm/s %.2f mm (ideal 11.0 + half the window)\n", ramp); check(fabs(ramp-11.09) < 0.2, "acceleration 1000 mm/s^2"); }
  send("MOVE -40 5 @4"); run_until("start",0.1); run_until(nullptr,1.0);
  send("STOP @5"); std::string m = run_until("id=5",1);
  check(m.find("stopped pos=")!=std::string::npos && m.find("id=4")!=std::string::npos, "STOP: the stopper and the mover both hear stopped");
  { int64_t at_reply = stage_pos; run_until(nullptr,0.3); check(stage_pos == at_reply, "stopped is sent once the stage stands still"); }
  check(stage_pos == real - real0, "no step lost or invented while stopping");
  send("STATUS @6"); check(run_until("status",0.1).find("status homed=1 moving=0")!=std::string::npos, "STATUS after the stop");
  send("GOTO 200 10 @7"); check(run_until("error",0.1).find("out of range")!=std::string::npos, "GOTO beyond the travel is refused");
  send("MOVE 1 500"); check(run_until("error",0.1).find("error speed")!=std::string::npos, "too fast is refused");
  send("MOVE 1"); check(run_until("error",0.1).find("usage")!=std::string::npos, "bad command is refused");
  send("GOTO 90 50 @8"); run_until("start",0.1); run_until(nullptr,0.3); power=false;
  check(run_until("power",0.2).find("error motor power off id=8")!=std::string::npos, "motor power cut during a move is reported");
  send("GOTO 10 50 @9"); check(run_until("error",0.1).find("power off id=9")!=std::string::npos, "no move without motor power");
  power=true;
  send("MOVE 1 5 @13"); check(run_until("error",0.1).find("not homed id=13")!=std::string::npos, "home is forgotten after a power cut");
  send("HOME @10"); check(run_until("homed",30).find("homed id=10")!=std::string::npos, "HOME again");
  real0 = real;
  MAX_AT = 1600*6;  // far switch moved close: hit it
  send("GOTO 90 20 @11"); check(run_until("error",10).find("limit switch")!=std::string::npos, "far switch stops the move");
  int64_t at_switch = real; run_until(nullptr,0.2);
  check(real == at_switch && digitalRead(8)==HIGH && real - (MAX_AT+HOME_AT) == 0, "no step into the pressed switch");
  MAX_AT = 1600*102;
  min_broken = true; send("HOME @12");
  check(run_until("error",30).find("home switch not found id=12")!=std::string::npos, "broken home switch wire: HOME fails");
  min_broken = false;
  size_t before = Serial.out.size();
  for(int i=0;i<12;i++) send("STATUS");
  run_until(nullptr,0.05);
  int answers=0; for(size_t p=Serial.out.find("# stage: status",before); p!=std::string::npos; p=Serial.out.find("# stage: status",p+1)) answers++;
  check(answers==12 && Serial.out.find("queue full",before)==std::string::npos, "a burst of 12 commands is all answered");
  printf("%s\n", failures ? "FAILED" : "all passed");
  return failures ? 1 : 0;
}
