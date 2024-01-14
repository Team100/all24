#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H

#include <stdint.h>
#include <Arduino.h>
#include "PluggableUSB.h"
#include "Data.h"

/**
 * HID Report Descriptor
 *
 * This layout must match the struct in ReportTx.
 *
 * Configuring the 9th axis ("wheel") confuses the DS, so skip it.
 *
 * There's no report id here, so every report should include everything.
 *
 * See hut1_3_0.pdf section 4 for details
 */
static const uint8_t HIDReportDescriptor[] = {
  0x05, 0x01,        // Usage Page: Generic Desktop Controls (0x01)
  0x09, 0x04,        // Usage: Joystick (0x04)
  0xa1, 0x01,        // Collection type: Application (0x01)
                     // Joysticks
  0x05, 0x01,        // ....Usage Page: Generic Desktop Controls (0x01)
  0x09, 0x01,        // ....Usage: Pointer (0x01)v
  0x16, 0x00, 0x80,  // ....Logical minimum: -32768
  0x26, 0xff, 0x7f,  // ....Logical maximum: 32767
  0x75, 0x10,        // ....Report size: 16
  0x95, 0x08,        // ....Report count: 8
  0xa1, 0x00,        // ....Collection type: Physical (0x00)
  0x09, 0x30,        // ........Usage: X (0x30)
  0x09, 0x31,        // ........Usage: Y (0x31)
  0x09, 0x32,        // ........Usage: Z (0x32)
  0x09, 0x33,        // ........Usage: Rx (0x33)
  0x09, 0x34,        // ........Usage: Ry (0x34)
  0x09, 0x35,        // ........Usage: Rz (0x35)
  0x09, 0x36,        // ........Usage: Slider (0x36)
  0x09, 0x37,        // ........Usage: Dial (0x37)
  0x81, 0x02,        // ........Input (Data,Var,Abs)
  0xc0,              // ....End Collection (0xc)
                     // Buttons
  0x05, 0x09,        // ....Usage Page: Button (0x09)
  0x19, 0x01,        // ....Usage minimum: 0x01
  0x29, 0x20,        // ....Usage maximum: 0x20
  0x15, 0x00,        // ....Logical minimum: 0
  0x25, 0x01,        // ....Logical maximum: 1
  0x75, 0x01,        // ....Report size: 1
  0x95, 0x20,        // ....Report count: 32
  0x81, 0x02,        // ....Input (Data,Var,Abs)
                     // Outputs (the DS appears to support 32 bits but only works with 16.)
  0x05, 0x08,        // ....Usage Page: LED (0x08)
  0x19, 0x01,        // ....Usage minimum: 0x01
  0x29, 0x10,        // ....Usage maximum: 16 (32 won't be populated)
  0x15, 0x00,        // ....Logical minimum: 0
  0x25, 0x01,        // ....Logical maximum: 1
  0x75, 0x01,        // ....Report size: 1
  0x95, 0x10,        // ....Report count: 16 (32 does not work)
  0x91, 0x02,        // ....Output (Data,Var,Abs)
  0xc0,              // End Collection (0xc)
};

static const char *MANUFACTURER_DESCRIPTOR = "Team 100";
static const char *SERIAL_NUMBER = "SerialNumber";

/**
 * Sends and receives data via USB.
*/
class Transceiver : public PluggableUSBModule {
public:

  /**
   * Each submodule sketch should use one of these.
   *
   * The value is the "product id" used in the USB Device Descriptor.
   * Windows assumes that the product id/product name association never
   * changes, so it caches it in the registry forever.  To avoid confusion
   * new names should never be assigned to old enum values.
   */
  enum class SubConsole : uint16_t {
    PILOT = 3,
    AUTOPILOT = 4,
    CLIMB = 5,
    ARM = 6,
    BUTTONS = 7,
    DEBUG = 8,
    KNOBS = 9,
    MIDI = 10,
    MIDI_DUAL = 11
  };

  /**
   * Never change these strings without also changing the numbers above.
   */
  const char *getProductDescriptor() {
    switch (subConsole_) {
      case SubConsole::PILOT: return "Pilot";
      case SubConsole::AUTOPILOT: return "Autopilot";
      case SubConsole::CLIMB: return "Climb";
      case SubConsole::ARM: return "Arm";
      case SubConsole::BUTTONS: return "Button Board";
      case SubConsole::DEBUG: return "Debug";
      case SubConsole::KNOBS: return "Knobs";
      case SubConsole::MIDI: return "MIDI";
      case SubConsole::MIDI_DUAL: return "MIDI_DUAL";
      default: return "Unassigned";
    }
  }

  Transceiver(SubConsole subConsole, ReportRx &reportRx)
    : PluggableUSBModule(1, 1, epType),
      subConsole_(subConsole),
      reportRx_(reportRx) {
    epType[0] = 0xc1;  // endpoint type = interrupt in
    PluggableUSB().plug(this);
  }

  /** 
   * Sends the data as a HID Report.
   */
  void send(const ReportTx &reportTx) {
    // Ignore changes < 0.1%.
    // TODO: the proper tolerance depends on what these axes actually measure,
    // so let the caller decide.
    if (reportTx.approxEquals(previousReportTx_, 60)) {
      return;
    }
    USB_Send(pluggedEndpoint | TRANSFER_RELEASE, (const void *)&reportTx, sizeof(reportTx));
    previousReportTx_ = reportTx;
  }

protected:
  /**
  * Handles USB Class-specific requests using the Default pipe.
  *
  * Ignores all DeviceToHost requests (GetReport, GetIdle, GetProtocol).
  * Only the Interrupt In pipe is used for device-to-host messages.
  *
  * Ignores all HostToDevice requests (e.g. SetIdle, SetProtocol, SetFeature) except
  * for one, SetReport, which contains output data.  This method is intended for
  * higher-latency outputs compared to the Interrupt Out pipe, which is not used by the DS.
  *
  * Returns true if the request is handled, false if ignored.  Maybe that produces
  * a NAK?
  *
  * See hid1_1.pdf section 7.2 for details.
  */
  bool setup(USBSetup &setup) {
    if (setup.bmRequestType == 0x21          // request type = host to device
        && setup.bRequest == 0x09            // request = SET_REPORT
        && setup.wValueH == 0x02             // report type = OUTPUT
        && setup.wIndex == pluggedInterface  // The message is addressed to this interface.
        && setup.wLength == sizeof(reportRx_)) {
      USB_RecvControl((uint8_t *)&(reportRx_), setup.wLength);
      return true;
    }
    // Returning false indicates we're not listening, doesn't seem to hurt anything.
    return false;
  }

  /**
   * Supplies the HID details to the control channel.
   *
   * Uses pluggedInterface and pluggedEndpoint, which are
   * assigned in PluggableUSB().plug(), called in ctor.
   */
  int getInterface(uint8_t *interfaceCount) {
    *interfaceCount += 1;

    const uint8_t interfaceDescriptor[] = {
      // INTERFACE DESCRIPTOR (2.0): class HID
      0x09,              // bLength: 9
      0x04,              // bDescriptorType: 0x04 (INTERFACE)
      pluggedInterface,  // bInterfaceNumber
      0x00,              // bAlternateSetting: 0
      0x01,              // bNumEndpoints: 1
      0x03,              // bInterfaceClass: HID (0x03)
      0x00,              // bInterfaceSubClass: No Subclass (0x00)
      0x00,              // bInterfaceProtocol: 0x00
      0x04,              // iInterface descriptor index: 4
      // HID DESCRIPTOR
      // TODO: why this doesn't match the struct in HID.h?
      0x09,                                  // bLength: 9
      0x21,                                  // bDescriptorType: 0x21 (HID)
      0x01, 0x01,                            // bcdHID: 0x0101 (21)
      0x00,                                  // bCountryCode: Not Supported (0x00)
      0x01,                                  // bNumDescriptors: 1
      0x22,                                  // bDescriptorType: HID Report (0x22)
      lowByte(sizeof(HIDReportDescriptor)),  // wDescriptorLength
      highByte(sizeof(HIDReportDescriptor)),
      // ENDPOINT DESCRIPTOR
      0x07,                               // bLength: 7
      0x05,                               // bDescriptorType: 0x05 (ENDPOINT)
      (uint8_t)(pluggedEndpoint | 0x80),  // bEndpointAddress
      0x03,                               // bmAttributes: 0x03 (Transfertype: Interrupt-Transfer (0x3))
      0x40, 0x00,                         // wMaxPacketSize: 64
      0x01,                               // bInterval: 1

    };
    return USB_SendControl(0, interfaceDescriptor, sizeof(interfaceDescriptor));
  }

  /**
   * Cribbed from USBCore.cpp because it's not exposed in USBAPI.h. 
   * 
   * USB wants this to be UTF-16 but the string above is ASCII, so this kinda fixes it up.
   *
   * the response format is:
   * byte 1: the length of the payload including this header
   * byte 2: the number 3 which means "this is a string payload"
   * the rest of the bytes, UTF-16 encoded which just means zeros.
   *
   * we don't have access to the raw SendControl(u8) function so we have to
   * create a payload for USB_SendControl(void* d, len) function.
   */
  bool SendStringDescriptor(const char *string_P, u8 string_len) {
    u8 buflen = 2 + string_len * 2;
    char buffer[buflen];
    buffer[0] = buflen;
    buffer[1] = 0x03;
    for (u8 i = 0; i < string_len; i++) {
      buffer[2 + i * 2] = string_P[i];
      buffer[3 + i * 2] = 0;
    }
    return USB_SendControl(0, buffer, buflen);
  }

  /**
 * USB Device Descriptor
 *
 * This provides vendor id and product id, which Windows uses as keys.
 *
 * See https://www.usb.org/defined-class-codes for device class/subclass/protocols
 * See http://www.linux-usb.org/usb-ids.html for vendor and product ids.
 *
 * The original Leonardo values are
 * 0x41, 0x23, // idVendor: 0x2341 (Arduino)
 * 0x36, 0x80, // idProduct: 0x8036 (Leonardo)
 */
  int sendUSBDeviceDescriptor() {
    const uint8_t USBDeviceDescriptor[] = {
      0x12,                                          // bLength: 18
      0x01,                                          // bDescriptorType: 1 (device)
      0x02, 0x00,                                    // bcdUSB: 2
      0xef,                                          // bDeviceClass: miscellaneous
      0x02,                                          // bDeviceSubClass: 2
      0x01,                                          // bDeviceProtocol: 1 (interface association descriptor)
      0x40,                                          // bMaxPacketSize: 64
      0x43, 0x23,                                    // idVendor: 0x2343 (unassigned)
      lowByte(static_cast<uint16_t>(subConsole_)),   // idProduct (l)
      highByte(static_cast<uint16_t>(subConsole_)),  // idProduct (h)
      0x00, 0x01,                                    // bcdDevice: 0x0100, release number
      0x01,                                          // iManufacturer descriptor index = 1
      0x02,                                          // iProduct descriptor index = 2
      0x03,                                          // iSerialNumber descriptor index = 3
      0x01                                           // bNumConfigurations
    };
    return USB_SendControl(0, USBDeviceDescriptor, sizeof(USBDeviceDescriptor));
  }

  /**
   * Handles GetDescriptor requests.
   *
   * Returns the number of bytes sent.
   *
   * See https://www.usb.org/sites/default/files/documents/hid1_11.pdf section 7.1 for details.
   */
  int getDescriptor(USBSetup &setup) {
    if (setup.bmRequestType == 0x80) {  // Request type = standard
      if (setup.wValueH == 0x01) {      // Descriptor type = device
        return sendUSBDeviceDescriptor();
      } else if (setup.wValueH == 0x03) {  // Descriptor type = string
        if (setup.wValueL == 0x01) {       // Descriptor index = manufacturer
          return SendStringDescriptor(MANUFACTURER_DESCRIPTOR, strlen(MANUFACTURER_DESCRIPTOR));
        } else if (setup.wValueL == 0x02) {  // Descriptor index = product
          const char *productDescriptor = getProductDescriptor();
          return SendStringDescriptor(productDescriptor, strlen(productDescriptor));
        } else if (setup.wValueL == 0x03) {  // Descriptor index = serial
          return SendStringDescriptor(SERIAL_NUMBER, strlen(SERIAL_NUMBER));
        } else if (setup.wValueL == 0x04) {  // Descriptor index = interface
          const char *productDescriptor = getProductDescriptor();
          return SendStringDescriptor(productDescriptor, strlen(productDescriptor));
        }
      }
    } else if (setup.bmRequestType == 0x81) {  // Request type = HID class descriptor
      if (setup.wIndex == pluggedInterface) {  // Interface number = this one
        if (setup.wValueH == 0x22) {           // Descriptor type = report
          return USB_SendControl(0, HIDReportDescriptor, sizeof(HIDReportDescriptor));
        }
      }
    }
    return 0;
  }

private:
  uint8_t epType[1];
  ReportRx &reportRx_;
  ReportTx previousReportTx_;
  SubConsole subConsole_;
};
#endif  // TRANSCEIVER_H
