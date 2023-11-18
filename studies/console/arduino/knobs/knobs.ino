#include "Data.h"
#include "Transceiver.h"
#include "Sensor.h"

ReportRx reportRx_; // transceiver writes received data here, indicator displays it.
ReportTx reportTx_; // sensor writes data here, transceiver sends it.
Sensor sensor_;
Transceiver transceiver_(Transceiver::SubConsole::KNOBS, reportRx_);

void setup() {
  sensor_.initialize();
}

void loop() {
  sensor_.sense(reportTx_);
  transceiver_.send(reportTx_);
}