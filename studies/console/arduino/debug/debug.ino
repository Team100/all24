#include "Data.h"
#include "Transceiver.h"
#include "Sensor.h"

const static uint32_t LONG = 500;
const static uint32_t SHORT = 50;
const static uint8_t LED_PIN = 17;
uint8_t ledState = LOW;
uint32_t prevTime = 0;


ReportTx reportTx; 
ReportRx reportRx;
Transceiver transceiver(Transceiver::SubConsole::DEBUG, reportRx);
Sensor sensor;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  sensor.initialize();
}

void loop() {
  uint32_t curTime = millis();
  uint32_t interval = sensor.initialized ? LONG : SHORT;
  if (curTime - prevTime >= interval) {
    prevTime = curTime;
    ledState = (ledState == LOW)? HIGH : LOW;
    digitalWrite(LED_PIN, ledState);
  }
  
  sensor.sense(reportTx);
  transceiver.send(reportTx);
  //sensor.indicate(reportRx);
}