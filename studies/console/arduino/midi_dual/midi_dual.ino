#include "Data.h"
#include "Transceiver.h"
#include <MIDI.h>

ReportRx reportRx0_; // received data, could be used for MIDI OUT
ReportRx reportRx1_; // received data, could be used for MIDI OUT
ReportTx reportTx0_; // data to transmit
ReportTx reportTx1_; // data to transmit
Transceiver transceiver0_(Transceiver::SubConsole::MIDI_DUAL, reportRx0_);
Transceiver transceiver1_(Transceiver::SubConsole::MIDI_DUAL, reportRx1_);

MIDI_CREATE_DEFAULT_INSTANCE();

// splits the keyboard in half at F#4
void handleNote(byte pitch, bool on) {
  switch (pitch) {
    case 36: // C2
      reportTx0_.b1 = on;
      break;
    case 37: // C#2
      reportTx0_.b2 = on;
      break;
    case 38: // D2
      reportTx0_.b3 = on;
      break;
    case 39: // D#2
      reportTx0_.b4 = on;
      break;
    case 40: // E2
      reportTx0_.b5 = on;
      break;
    case 41: // F2
      reportTx0_.b6 = on;
      break;
    case 42: // F#2
      reportTx0_.b7 = on;
      break;
    case 43: // G2
      reportTx0_.b8 = on;
      break;
    case 44: // G#2
      reportTx0_.b9 = on;
      break;
    case 45: // A2
      reportTx0_.b10 = on;
      break;
    case 46: // A#2
      reportTx0_.b11 = on;
      break;
    case 47: // B2
      reportTx0_.b12 = on;
      break;

    case 48: // C3
      reportTx0_.b13 = on;
      break;
    case 49: // C#3
      reportTx0_.b14 = on;
      break;
    case 50: // D3
      reportTx0_.b15 = on;
      break;
    case 51: // D#3
      reportTx0_.b16 = on;
      break;
    case 52: // E3
      reportTx0_.b17 = on;
      break;
    case 53: // F3
      reportTx0_.b18 = on;
      break;
    case 54: // F#3
      reportTx0_.b19 = on;
      break;
    case 55: // G3
      reportTx0_.b20 = on;
      break;
    case 56: // G#3
      reportTx0_.b21 = on;
      break;
    case 57: // A3
      reportTx0_.b22 = on;
      break;
    case 58: // A#3
      reportTx0_.b23 = on;
      break;
    case 59: // B3
      reportTx0_.b24 = on;
      break;

    case 60: // C4
      reportTx0_.b25 = on;
      break;
    case 61: // C#4
      reportTx0_.b26 = on;
      break;
    case 62: // D4
      reportTx0_.b27 = on;
      break;
    case 63: // D#4
      reportTx0_.b28 = on;
      break;
    case 64: // E4
      reportTx0_.b29 = on;
      break;
    case 65: // F4
      reportTx0_.b30 = on;
      break;
    case 66: // F#4
      // unassigned
      break;
    case 67: // G4
      reportTx1_.b1 = on;
      break;
    case 68: // G#4
      reportTx1_.b2 = on;
      break;
    case 69: // A4
      reportTx1_.b3 = on;
      break;
    case 70: // A#4
      reportTx1_.b4 = on;
      break;
    case 71: // B4
      reportTx1_.b5 = on;
      break;

    case 72: // C5
      reportTx1_.b6 = on;
      break;
    case 73: // C#5
      reportTx1_.b7 = on;
      break;
    case 74: // D5
      reportTx1_.b8 = on;
      break;
    case 75: // D#5
      reportTx1_.b9 = on;
      break;
    case 76: // E5
      reportTx1_.b10 = on;
      break;
    case 77: // F5
      reportTx1_.b11 = on;
      break;
    case 78: // F#5
      reportTx1_.b12 = on;
      break;
    case 79: // G5
      reportTx1_.b13 = on;
      break;
    case 80: // G#5
      reportTx1_.b14 = on;
      break;
    case 81: // A5
      reportTx1_.b15 = on;
      break;
    case 82: // A#5
      reportTx1_.b16 = on;
      break;
    case 83: // B5
      reportTx1_.b17 = on;
      break;

    case 84: // C6
      reportTx1_.b18 = on;
      break;
    case 85: // C#6
      reportTx1_.b19 = on;
      break;
    case 86: // D6
      reportTx1_.b20 = on;
      break;
    case 87: // D#6
      reportTx1_.b21 = on;
      break;
    case 88: // E6
      reportTx1_.b22 = on;
      break;
    case 89: // F6
      reportTx1_.b23 = on;
      break;
    case 90: // F#6
      reportTx1_.b24 = on;
      break;
    case 91: // G6
      reportTx1_.b25 = on;
      break;
    case 92: // G#6
      reportTx1_.b26 = on;
      break;
    case 93: // A6
      reportTx1_.b27 = on;
      break;
    case 94: // A#6
      reportTx1_.b28 = on;
      break;
    case 95: // B6
      reportTx1_.b29 = on;
      break;

    case 96: // C7
      reportTx1_.b30 = on;
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
  transceiver0_.send(reportTx0_);
  transceiver1_.send(reportTx1_);
}
