#include "Data.h"
#include "Transceiver.h"
#include "Sensor.h"

const static uint32_t LONG = 500;
const static uint32_t SHORT = 50;
const static uint8_t LED_PIN = 17;
uint8_t ledState = LOW;
uint32_t prevTime = 0;

ReportRx reportRx;  // transceiver writes received data here, indicator displays it.
ReportTx reportTx;  // sensor writes data here, transceiver sends it.
Transceiver transceiver(Transceiver::SubConsole::PILOT, reportRx);
Sensor sensor;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  sensor.initialize();
  sensor.splash();
}


void loop() {
  // TODO: make the pattern flasher a library
  uint32_t curTime = millis();
  uint32_t interval = sensor.initialized ? LONG : SHORT;
  if (curTime - prevTime >= interval) {
    prevTime = curTime;
    ledState = (ledState == LOW)? HIGH : LOW;
    digitalWrite(LED_PIN, ledState);
  }

  sensor.sense(reportTx);
  transceiver.send(reportTx);
  ///////////////////////
  // for this demo we loop back.
  // TODO: take this out, do it in the RIO.
  ///////////////////////
  reportRx = *(ReportRx*)((char*)(&reportTx) + 16);
  ///////////////////////
  // TODO: remove the above line
  ///////////////////////
  sensor.indicate(reportRx);
}