#include "Data.h"
#include "Transceiver.h"
#include <MIDI.h>

ReportRx reportRx_; // received data, could be used for MIDI OUT
ReportTx reportTx_; // data to transmit
Transceiver transceiver_(Transceiver::SubConsole::MIDI, reportRx_);

MIDI_CREATE_DEFAULT_INSTANCE();

// all the white keys from C2 to F6
void handleNote(byte pitch, bool on) {
  switch (pitch) {
    case 36: // C2
      reportTx_.b1 = on;
      break;
    case 38: // D2
      reportTx_.b2 = on;
      break;
    case 40: // E2
      reportTx_.b3 = on;
      break;
    case 41: // F2
      reportTx_.b4 = on;
      break;
    case 43: // G2
      reportTx_.b5 = on;
      break;
    case 45: // A2
      reportTx_.b6 = on;
      break;
    case 47: // B2
      reportTx_.b7 = on;
      break;

    case 48: // C3
      reportTx_.b8 = on;
      break;
    case 50: // D3
      reportTx_.b9 = on;
      break;
    case 52: // E3
      reportTx_.b10 = on;
      break;
    case 53: // F3
      reportTx_.b11 = on;
      break;
    case 55: // G3
      reportTx_.b12 = on;
      break;
    case 57: // A3
      reportTx_.b13 = on;
      break;
    case 59: // B3
      reportTx_.b14 = on;
      break;

    case 60: // C4
      reportTx_.b15 = on;
      break;
    case 62: // D4
      reportTx_.b16 = on;
      break;
    case 64: // E4
      reportTx_.b17 = on;
      break;
    case 65: // F4
      reportTx_.b18 = on;
      break;
    case 67: // G4
      reportTx_.b19 = on;
      break;
    case 69: // A4
      reportTx_.b20 = on;
      break;
    case 71: // B4
      reportTx_.b21 = on;
      break;

    case 72: // C5
      reportTx_.b22 = on;
      break;
    case 74: // D5
      reportTx_.b23 = on;
      break;
    case 76: // E5
      reportTx_.b24 = on;
      break;
    case 77: // F5
      reportTx_.b25 = on;
      break;
    case 79: // G5
      reportTx_.b26 = on;
      break;
    case 81: // A5
      reportTx_.b27 = on;
      break;
    case 83: // B5
      reportTx_.b28 = on;
      break;

    case 84: // C6
      reportTx_.b29 = on;
      break;
    case 86: // D6
      reportTx_.b30 = on;
      break;
    case 88: // E6
      reportTx_.b31 = on;
      break;
    case 89: // F6
      reportTx_.b32 = on;
      break;
  }
}

void handleNoteOn(byte channel, byte pitch, byte velocity) {
  handleNote(pitch, true);
}

void handleNoteOff(byte channel, byte pitch, byte velocity) {
  handleNote(pitch, false);
}

void setup() {
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.begin(MIDI_CHANNEL_OMNI);
}

void loop() {
  MIDI.read();
  transceiver_.send(reportTx_);
}
