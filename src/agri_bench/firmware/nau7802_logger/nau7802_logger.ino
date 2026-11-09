// Load cell logger for agri_bench's force_logger (docs/stem_force_test_plan.md).
//
// Board: Raspberry Pi Pico 2 with the arduino-pico core (other Arduino
// boards with I2C work too). Libraries: "SparkFun Qwiic Scale NAU7802".
//
// Wiring:
//   NAU7802 boards (SparkFun SEN-15242), one per load cell. They all have
//   the I2C address 0x2A, so each sits on its own port of a TCA9548A I2C
//   multiplexer (address 0x70) on Wire (Pico: GP4 SDA, GP5 SCL).
//   MUX_PORTS lists the ports in the order of force_logger's channels
//   parameter (default: cut, grip_left, grip_right).
//   Optional linear potentiometer for the blade / finger travel on
//   POSITION_PIN (Pico: A0 = GP26, 0-3.3 V). Set it to -1 when unused and
//   drop 'position' from the channels parameter.
//
// Output, 115200 baud, one line each time the first channel has a new
// sample (80 Hz); the others send their latest reading:
//   D,<micros>,<raw cut>,<raw grip_left>,<raw grip_right>[,<raw position>]
// Lines starting with '#' are information or errors. If any load cell
// does not answer at start-up, the board sends no data, only an error line
// every second, so a missing sensor can never look like a zero force.
//
// Not yet tried on hardware: check the header line and the raw values with
// a serial monitor before the first trial.

#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

const uint8_t MUX_ADDR = 0x70;
const uint8_t MUX_PORTS[] = {0, 1, 2};
const int CHANNELS = sizeof(MUX_PORTS) / sizeof(MUX_PORTS[0]);
const int POSITION_PIN = A0;   // -1: no position channel

NAU7802 scales[CHANNELS];
int32_t latest[CHANNELS];
bool ok[CHANNELS];
bool all_ok = true;
bool seen[CHANNELS];   // has sent at least one reading

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
    scales[i].setSampleRate(NAU7802_SPS_80);
    scales[i].calibrateAFE();
  }
  Serial.print("# agri_bench nau7802_logger channels=");
  Serial.print(CHANNELS + (POSITION_PIN >= 0 ? 1 : 0));
  Serial.println(POSITION_PIN >= 0 ? " position=yes sps=80" : " position=no sps=80");
}

void loop() {
  if (!all_ok) {
    for (int i = 0; i < CHANNELS; i++) {
      if (!ok[i]) {
        Serial.print("# error: no NAU7802 on multiplexer port ");
        Serial.println(MUX_PORTS[i]);
      }
    }
    delay(1000);
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
  Serial.println();
}
