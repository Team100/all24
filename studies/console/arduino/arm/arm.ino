#include "Data.h"
#include "Transceiver.h"
#include "Sensor.h"
#include "Indicator.h"

Data data_;
Transceiver transceiver_(Transceiver::SubConsole::ARM, data_);
Sensor sensor_(data_);
Indicator indicator_(data_.reportRx_);

void setup() {
}

void loop() {
  if (sensor_.sense()) transceiver_.send();
  if (transceiver_.recv()) indicator_.indicate();
}