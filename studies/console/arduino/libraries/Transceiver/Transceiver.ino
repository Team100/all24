// This sketch makes it compile :-)
#include "Data.h"
#include "Transceiver.h"

ReportRx reportRx_;
Transceiver transceiver_(Transceiver::SubConsole::PILOT, reportRx_);
void setup() {}
void loop() {}