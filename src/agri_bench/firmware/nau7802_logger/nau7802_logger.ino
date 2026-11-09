// Load cell logger and linear stage driver for agri_bench's force_logger
// (docs/stem_force_test_plan.md, docs/stem_force_bench_bom.md).
//
// Board: Raspberry Pi Pico 2 with the arduino-pico core (it runs setup1 /
// loop1 on the second core). Library: "SparkFun Qwiic Scale NAU7802".
//
// Core 0 reads the load cells, takes the stage commands from USB serial
// and sends the data lines. Core 1 only makes the stepper pulses, so the
// stage moves at an even speed whatever the I2C reads take, and its
// position goes out in the same lines, on the same clock, as the forces.
//
// Wiring:
//   NAU7802 boards (SparkFun SEN-15242), one per load cell. They all have
//   the I2C address 0x2A, so each sits on its own port of a TCA9548A /
//   PCA9548 I2C multiplexer (address 0x70) on Wire (Pico: GP4 SDA, GP5
//   SCL). MUX_PORTS lists the ports in the order of force_logger's
//   channels parameter (default: cut, grip_left, grip_right).
//   Optional linear potentiometer for the handle travel on POSITION_PIN
//   (Pico: A0 = GP26, 0-3.3 V). Set it to -1 when unused and drop
//   'position' from the channels parameter.
//   Stage (STAGE = true; else drop 'stage' from the channels parameter):
//     TMC2209 driver: STEP_PIN, DIR_PIN, ENABLE_PIN (low = on). MS1 and
//     MS2 left low give 8 microsteps (STEPS_PER_MM below).
//     Limit switches, normally closed, between LIMIT_MIN_PIN /
//     LIMIT_MAX_PIN and GND: a pressed switch or a broken wire both read
//     high and stop the stage. The home switch is LIMIT_MIN.
//     MOTOR_POWER_PIN: the motor supply through a divider to below 3.3 V
//     (24 V: 100 k to the pin, 15 k to GND, about 3.1 V). The emergency
//     stop cuts that supply; the board then stops counting steps,
//     forgets its home and refuses moves until HOME. -1: not wired (the
//     board then cannot tell that the position was lost: home again by
//     hand after every emergency stop).
//
// Output, 115200 baud, one line each time the first load cell has a new
// sample (SPS, 320 Hz); the others send their latest reading:
//   D,<micros>,<raw cut>,<raw grip_left>,<raw grip_right>[,<raw position>][,<stage steps>]
// Lines starting with '#' are information or errors. If any load cell
// does not answer at start-up, the board sends no data, only an error line
// every second, so a missing sensor can never look like a zero force (the
// stage still takes commands).
//
// Stage commands, one line each (see agri_bench/stage.py):
//   HOME | MOVE <mm> <mm/s> | GOTO <mm> <mm/s> | STOP | STATUS, each
//   optionally ending in @<id> (a number)
// and their replies, '# stage: ...' lines, with id=<id> when the command
// had one (also on the done / stopped / error that ends its move).
//
// Not yet tried on hardware: check the header line and the raw values with
// a serial monitor before the first trial, and run HOME with a hand on the
// emergency stop.

#include <Wire.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

// ---- load cells (core 0) ----
const uint8_t MUX_ADDR = 0x70;
const uint8_t MUX_PORTS[] = {0, 1, 2};
const int CHANNELS = sizeof(MUX_PORTS) / sizeof(MUX_PORTS[0]);
const int POSITION_PIN = A0;   // -1: no position channel
// 320 samples/s: at the fast cut (50 mm/s at the blade) a stem parts in a
// few hundredths of a second, and 80/s would miss most of the peak. Use
// NAU7802_SPS_80 (and SPS = 80) for slow tests if the noise matters more.
const uint8_t SPS_SETTING = NAU7802_SPS_320;
const int SPS = 320;

// ---- stage (core 1) ----
const bool STAGE = true;
const int STEP_PIN = 2;
const int DIR_PIN = 3;
const int ENABLE_PIN = 6;
const int LIMIT_MIN_PIN = 7;    // home switch
const int LIMIT_MAX_PIN = 8;
const int MOTOR_POWER_PIN = 9;  // -1: not wired
const bool DIR_INVERT = false;  // true if + moves toward the home switch
const float STEPS_PER_MM = 200.0f * 8.0f / 10.0f;  // full steps/turn * microsteps / lead [mm]
const float ACCEL_MM_S2 = 1000.0f;
const float MAX_SPEED_MM_S = 200.0f;
const float TRAVEL_MM = 100.0f;      // soft limit: 0..TRAVEL_MM after HOME
const float HOME_SPEED_MM_S = 10.0f;
const float HOME_BACKOFF_MM_S = 2.0f;

NAU7802 scales[CHANNELS];
int32_t latest[CHANNELS];
bool ok[CHANNELS];
bool all_ok = true;
bool seen[CHANNELS];   // has sent at least one reading
uint32_t last_error_ms = 0;

// Both directions use a single-producer, single-consumer ring: the
// producer fills a slot, then (after a barrier) moves head; the consumer
// reads the slot, then moves tail. Nothing is overwritten: a full command
// ring refuses the command, a full reply ring counts what it dropped.

// Commands, core 0 -> core 1, in the order received.
enum Command { CMD_HOME, CMD_MOVE, CMD_GOTO, CMD_STOP, CMD_STATUS };
const int COMMANDS = 8;
volatile uint8_t cmd_type[COMMANDS];
volatile float cmd_mm[COMMANDS], cmd_speed[COMMANDS];
volatile uint32_t cmd_id[COMMANDS];
volatile uint32_t cmd_head = 0, cmd_tail = 0;

// Replies, core 1 -> core 0.
enum Event {
  EV_START, EV_DONE, EV_STOPPED, EV_HOMED, EV_STATUS, EV_ERR_NOT_HOMED, EV_ERR_RANGE,
  EV_ERR_SPEED, EV_ERR_BUSY, EV_ERR_LIMIT, EV_ERR_HOME, EV_ERR_POWER
};
const int EVENTS = 32;
volatile uint8_t ev_code[EVENTS];
volatile int32_t ev_pos[EVENTS], ev_arg[EVENTS];
volatile uint32_t ev_id[EVENTS];
volatile uint32_t ev_head = 0, ev_tail = 0;
volatile uint32_t ev_dropped = 0;

volatile int32_t stage_pos = 0;   // [steps], written by core 1 only
volatile bool homed = false;
volatile bool moving = false;

// ===================== core 0 =====================

void selectPort(uint8_t port) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << port);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }
  Wire.begin();
  Wire.setClock(400000);
  if (POSITION_PIN >= 0) {
    analogReadResolution(12);
  }
  for (int i = 0; i < CHANNELS; i++) {
    selectPort(MUX_PORTS[i]);
    ok[i] = scales[i].begin(Wire);
    latest[i] = 0;
    seen[i] = false;
    if (!ok[i]) {
      all_ok = false;
      continue;
    }
    scales[i].setGain(NAU7802_GAIN_128);
    scales[i].setSampleRate(SPS_SETTING);
    scales[i].calibrateAFE();
  }
  Serial.print("# agri_bench nau7802_logger channels=");
  Serial.print(CHANNELS + (POSITION_PIN >= 0 ? 1 : 0) + (STAGE ? 1 : 0));
  Serial.print(POSITION_PIN >= 0 ? " position=yes" : " position=no");
  Serial.print(STAGE ? " stage=yes" : " stage=no");
  Serial.print(" sps=");
  Serial.println(SPS);
}

float mm(int32_t steps) {
  return steps / STEPS_PER_MM;
}

void reply(const char *text);

void sendCommand(int type, float a, float b, uint32_t id) {
  uint32_t h = cmd_head;
  if (h - cmd_tail >= (uint32_t)COMMANDS) {
    reply("error busy: command queue full");
    return;
  }
  int i = h % COMMANDS;
  cmd_type[i] = type;
  cmd_mm[i] = a;
  cmd_speed[i] = b;
  cmd_id[i] = id;
  __sync_synchronize();
  cmd_head = h + 1;
}

void reply(const char *text) {
  Serial.print("# stage: ");
  Serial.println(text);
}

bool parseNumber(const char *word, float *out) {
  if (word == NULL) {
    return false;
  }
  char *end;
  *out = strtof(word, &end);
  return end != word && *end == '\0' && isfinite(*out);
}

// Split a line into at most 4 words and an optional trailing @<id>.
int splitLine(char *line, char *words[], uint32_t *id) {
  int n = 0;
  *id = 0;
  for (char *w = strtok(line, " \t"); w != NULL; w = strtok(NULL, " \t")) {
    if (w[0] == '@') {
      char *end;
      unsigned long v = strtoul(w + 1, &end, 10);
      if (end != w + 1 && *end == '\0') {
        *id = (uint32_t)v;
        continue;
      }
    }
    if (n == 4) {
      return -1;
    }
    words[n++] = w;
  }
  return n;
}

void replyId(const char *text, uint32_t id) {
  Serial.print("# stage: ");
  Serial.print(text);
  if (id != 0) {
    Serial.print(" id=");
    Serial.print(id);
  }
  Serial.println();
}

void handleLine(char *line) {
  char *words[4];
  uint32_t id;
  int n = splitLine(line, words, &id);
  if (n == 0) {
    return;
  }
  if (!STAGE) {
    replyId("error no stage", id);
    return;
  }
  if (n < 0) {
    replyId("error usage", id);
    return;
  }
  const char *name = words[0];
  if (n == 1 && strcmp(name, "HOME") == 0) {
    sendCommand(CMD_HOME, 0, 0, id);
  } else if (n == 1 && strcmp(name, "STOP") == 0) {
    sendCommand(CMD_STOP, 0, 0, id);
  } else if (n == 1 && strcmp(name, "STATUS") == 0) {
    sendCommand(CMD_STATUS, 0, 0, id);
  } else if (strcmp(name, "MOVE") == 0 || strcmp(name, "GOTO") == 0) {
    float a, b;
    if (n != 3 || !parseNumber(words[1], &a) || !parseNumber(words[2], &b)) {
      replyId("error usage: MOVE|GOTO <mm> <mm/s>", id);
      return;
    }
    sendCommand(name[0] == 'M' ? CMD_MOVE : CMD_GOTO, a, b, id);
  } else {
    replyId("error unknown command", id);
  }
}

void readCommands() {
  static char line[64];
  static int len = 0;
  static bool too_long = false;
  // At most one command per call, and none while core 1's queue is full:
  // the rest waits in the USB buffer instead of being refused.
  while (Serial.available() > 0 && cmd_head - cmd_tail < (uint32_t)COMMANDS) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      bool handled = false;
      if (too_long) {
        reply("error line too long");
      } else if (len > 0) {
        line[len] = '\0';
        handleLine(line);
        handled = true;
      }
      len = 0;
      too_long = false;
      if (handled) {
        return;
      }
    } else if (len < (int)sizeof(line) - 1) {
      line[len++] = toupper((unsigned char)c);
    } else {
      too_long = true;
    }
  }
}

void printEvents() {
  static uint32_t dropped_seen = 0;
  if (ev_dropped != dropped_seen) {
    Serial.print("# stage: error lost replies=");
    Serial.println(ev_dropped - dropped_seen);
    dropped_seen = ev_dropped;
  }
  while (ev_tail != ev_head) {
    __sync_synchronize();
    int i = ev_tail % EVENTS;
    uint8_t code = ev_code[i];
    float pos = mm(ev_pos[i]);
    int32_t raw_arg = ev_arg[i];
    float arg = mm(raw_arg);
    uint32_t id = ev_id[i];
    __sync_synchronize();
    ev_tail = ev_tail + 1;
    Serial.print("# stage: ");
    switch (code) {
      case EV_START:
        Serial.print("start pos=");
        Serial.print(pos, 3);
        Serial.print(" target=");
        Serial.print(arg, 3);
        break;
      case EV_DONE:
        Serial.print("done pos=");
        Serial.print(pos, 3);
        break;
      case EV_STOPPED:
        Serial.print("stopped pos=");
        Serial.print(pos, 3);
        break;
      case EV_HOMED:
        Serial.print("homed");
        break;
      case EV_STATUS:
        Serial.print("status homed=");
        Serial.print(raw_arg & 1);
        Serial.print(" moving=");
        Serial.print((raw_arg >> 1) & 1);
        Serial.print(" pos=");
        Serial.print(pos, 3);
        break;
      case EV_ERR_NOT_HOMED:
        Serial.print("error not homed");
        break;
      case EV_ERR_RANGE:
        Serial.print("error out of range target=");
        Serial.print(arg, 3);
        break;
      case EV_ERR_SPEED:
        Serial.print("error speed");
        break;
      case EV_ERR_BUSY:
        Serial.print("error busy");
        break;
      case EV_ERR_LIMIT:
        Serial.print("error limit switch pos=");
        Serial.print(pos, 3);
        break;
      case EV_ERR_HOME:
        Serial.print("error home switch not found");
        break;
      case EV_ERR_POWER:
        Serial.print("error motor power off");
        break;
    }
    if (id != 0) {
      Serial.print(" id=");
      Serial.print(id);
    }
    Serial.println();
  }
}

void loop() {
  readCommands();
  printEvents();
  if (!all_ok) {
    if (millis() - last_error_ms >= 1000) {
      last_error_ms = millis();
      for (int i = 0; i < CHANNELS; i++) {
        if (!ok[i]) {
          Serial.print("# error: no NAU7802 on multiplexer port ");
          Serial.println(MUX_PORTS[i]);
        }
      }
    }
    return;
  }
  bool fresh = false;
  for (int i = 0; i < CHANNELS; i++) {
    selectPort(MUX_PORTS[i]);
    if (scales[i].available()) {
      latest[i] = scales[i].getReading();
      seen[i] = true;
      if (i == 0) {
        fresh = true;
      }
    }
  }
  if (!fresh) {
    return;
  }
  for (int i = 0; i < CHANNELS; i++) {
    if (!seen[i]) {
      return;   // no line until every load cell has a real reading
    }
  }
  Serial.print("D,");
  Serial.print(micros());
  for (int i = 0; i < CHANNELS; i++) {
    Serial.print(',');
    Serial.print(latest[i]);
  }
  if (POSITION_PIN >= 0) {
    Serial.print(',');
    Serial.print(analogRead(POSITION_PIN));
  }
  if (STAGE) {
    Serial.print(',');
    Serial.print(stage_pos);
  }
  Serial.println();
}

// ===================== core 1: stage =====================

enum State { IDLE, MOVING, HOME_SEEK, HOME_BACKOFF };
const float ACCEL = ACCEL_MM_S2 * STEPS_PER_MM;   // [steps/s^2]
const float VMIN = sqrtf(2.0f * ACCEL);           // first step's speed [steps/s]

State state = IDLE;
int dir = 1;
int32_t remaining = 0;
float v = 0, vmax = 0;
uint32_t next_us = 0;
float next_frac = 0;      // fraction of a microsecond carried to the next step
uint32_t motion_id = 0;   // id of the command that started the motion
uint32_t stop_id = 0;     // id of the STOP that is slowing it down
bool stopping = false;
bool power_was_on = true;

// (Plain int and no default arguments in the signatures: the Arduino
// builder puts its prototypes above the enum definitions.)
void emit(int code, int32_t arg, uint32_t id) {
  uint32_t h = ev_head;
  if (h - ev_tail >= (uint32_t)EVENTS) {
    ev_dropped = ev_dropped + 1;
    return;
  }
  int i = h % EVENTS;
  ev_code[i] = code;
  ev_pos[i] = stage_pos;
  ev_arg[i] = arg;
  ev_id[i] = id;
  __sync_synchronize();
  ev_head = h + 1;
}

// The end of a move slowed down by STOP: both its starter and the
// stopper are told, with the final position.
void emitStopped() {
  emit(EV_STOPPED, 0, motion_id);
  if (stop_id != motion_id) {
    emit(EV_STOPPED, 0, stop_id);
  }
}

bool switchPressed(int pin) {
  return digitalRead(pin) == HIGH;   // normally closed to GND
}

bool powerOn() {
  return MOTOR_POWER_PIN < 0 || digitalRead(MOTOR_POWER_PIN) == HIGH;
}

void finish() {
  state = IDLE;
  moving = false;
}

void startMotion(int32_t steps, float speed, int next) {
  dir = steps >= 0 ? 1 : -1;
  remaining = steps >= 0 ? steps : -steps;
  vmax = speed;
  v = fminf(VMIN, vmax);
  digitalWrite(DIR_PIN, ((dir > 0) != DIR_INVERT) ? HIGH : LOW);
  delayMicroseconds(5);
  next_us = micros();
  next_frac = 0;
  state = (State)next;
  moving = true;
}

// One step pulse. Speed follows the trapezoid: up at ACCEL, at most vmax,
// and down so that it reaches VMIN at the last step.
void pulse() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN, LOW);
  stage_pos = stage_pos + dir;
  remaining--;
  float up = sqrtf(v * v + 2.0f * ACCEL);
  float down = sqrtf(2.0f * ACCEL * (float)remaining);
  v = fminf(vmax, fminf(up, down));
  float floor_v = fminf(VMIN, vmax);
  if (v < floor_v) {
    v = floor_v;
  }
  float interval = 1e6f / v + next_frac;
  uint32_t whole = (uint32_t)interval;
  next_frac = interval - whole;
  next_us += whole;
}

void handleCommand(int type, float a, float speed, uint32_t id) {
  if (type == CMD_STATUS) {
    emit(EV_STATUS, (homed ? 1 : 0) | (moving ? 2 : 0), id);
    return;
  }
  if (type == CMD_STOP) {
    if (state == MOVING) {
      int32_t need = (int32_t)ceilf(v * v / (2.0f * ACCEL));
      if (remaining > need) {
        remaining = need;
      }
      stopping = true;
      stop_id = id;   // told once the stage stands still, with the mover
      if (remaining <= 0) {
        finish();
        emitStopped();
      }
    } else {
      bool homing = state != IDLE;
      finish();   // homing is slow: it stops at once
      if (homing) {
        homed = false;
        if (id != motion_id) {
          emit(EV_STOPPED, 0, motion_id);
        }
      }
      emit(EV_STOPPED, 0, id);
    }
    return;
  }
  if (state != IDLE) {
    emit(EV_ERR_BUSY, 0, id);
    return;
  }
  if (!powerOn()) {
    emit(EV_ERR_POWER, 0, id);
    return;
  }
  if (type == CMD_HOME) {
    homed = false;
    motion_id = id;
    stopping = false;
    emit(EV_START, 0, id);
    startMotion(-(int32_t)((TRAVEL_MM + 20.0f) * STEPS_PER_MM), HOME_SPEED_MM_S * STEPS_PER_MM,
                HOME_SEEK);
    return;
  }
  if (!homed) {
    emit(EV_ERR_NOT_HOMED, 0, id);
    return;
  }
  if (!(speed > 0.0f && speed <= MAX_SPEED_MM_S)) {
    emit(EV_ERR_SPEED, 0, id);
    return;
  }
  int32_t target = lroundf(a * STEPS_PER_MM) + (type == CMD_MOVE ? stage_pos : 0);
  if (target < 0 || target > lroundf(TRAVEL_MM * STEPS_PER_MM)) {
    emit(EV_ERR_RANGE, target, id);
    return;
  }
  motion_id = id;
  stopping = false;
  stop_id = 0;
  emit(EV_START, target, id);
  if (target == stage_pos) {
    emit(EV_DONE, 0, id);
    return;
  }
  startMotion(target - stage_pos, speed * STEPS_PER_MM, MOVING);
}

void setup1() {
  if (!STAGE) {
    return;
  }
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(ENABLE_PIN, LOW);   // driver on, holds the position
  pinMode(LIMIT_MIN_PIN, INPUT_PULLUP);
  pinMode(LIMIT_MAX_PIN, INPUT_PULLUP);
  if (MOTOR_POWER_PIN >= 0) {
    pinMode(MOTOR_POWER_PIN, INPUT);
  }
}

void loop1() {
  if (!STAGE) {
    delay(1000);
    return;
  }
  bool power = powerOn();
  if (!power && power_was_on) {
    uint32_t id = state != IDLE ? motion_id : 0;
    finish();
    homed = false;
    emit(EV_ERR_POWER, 0, id);
  }
  power_was_on = power;

  if (cmd_tail != cmd_head) {
    __sync_synchronize();
    int i = cmd_tail % COMMANDS;
    int type = cmd_type[i];
    float a = cmd_mm[i], b = cmd_speed[i];
    uint32_t id = cmd_id[i];
    __sync_synchronize();
    cmd_tail = cmd_tail + 1;
    handleCommand(type, a, b, id);
  }
  if (state == IDLE || !power || (int32_t)(micros() - next_us) < 0) {
    return;
  }
  // A step is due. Look at the switches first, so that no step is ever
  // made into a pressed (or disconnected) switch.
  bool min_hit = switchPressed(LIMIT_MIN_PIN);
  bool max_hit = switchPressed(LIMIT_MAX_PIN);
  switch (state) {
    case MOVING:
      if ((dir < 0 && min_hit) || (dir > 0 && max_hit)) {
        finish();
        homed = false;
        emit(EV_ERR_LIMIT, 0, motion_id);
        return;
      }
      break;
    case HOME_SEEK:
      if (max_hit) {   // wrong direction (DIR_INVERT) or a broken wire
        finish();
        emit(EV_ERR_LIMIT, 0, motion_id);
        return;
      }
      if (min_hit) {
        startMotion((int32_t)(5.0f * STEPS_PER_MM), HOME_BACKOFF_MM_S * STEPS_PER_MM,
                    HOME_BACKOFF);
        return;
      }
      break;
    case HOME_BACKOFF:
      if (max_hit) {
        finish();
        emit(EV_ERR_LIMIT, 0, motion_id);
        return;
      }
      if (!min_hit) {
        stage_pos = 0;
        homed = true;
        finish();
        emit(EV_HOMED, 0, motion_id);
        return;
      }
      break;
    case IDLE:
      return;
  }
  pulse();
  if (remaining <= 0) {
    State was = state;
    finish();
    if (was == MOVING) {
      if (stopping) {
        emitStopped();
      } else {
        emit(EV_DONE, 0, motion_id);
      }
    } else {
      emit(EV_ERR_HOME, 0, motion_id);   // the switch never came (or never released)
    }
  }
}
