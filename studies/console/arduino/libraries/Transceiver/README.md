# Transceiver

USB HID library for all subconsoles.

The "ino" file here does nothing except convince the Arduino IDE to edit the other files.

Some background about USB HID protocol:

When you plug in a USB device, the host notices and engages in a conversation
that yields a tree of descriptors:

* A device descriptor.  There is only one of these per port (ignoring the hub protocol).
  The device descriptor includes the "product id" which is the host key for the device type.
  The device descriptor includes the number of configurations (one).
* One or more configuration descriptors.  Configuration descriptors are constructed by
  Arduino's USBCore.cpp, using ConfigDescriptor from USBCore.h, which doesn't have much
  interesting content except for the number of interfaces (in our case: one, except for
  the dual midi, which has two).
* One or more interface descriptors.  The interface descriptor includes the number
  of endpoints (in our case: one), and some description of them,
  e.g. a HID descriptor which includes the HID report descriptor, and an endpoint
  descriptor.
* Zero or more endpoints.

Some of these descriptor fields have string labels which are referenced by index,
e.g. the Device Descriptor field "iManufacturer" indicates index 1, which means 
that the manufacturer string can be fetched by index 1.  

The HID Report Descriptor is the description of the data that can be sent by the
device, and this corresponds, in our case, to the maximum amount of data that
can be understood by the Driver Station: 8 axes, 32 buttons, 16 outputs.

The important thing about this schema is that the product name is a device field,
so a single USB plug cannot (without acting like a hub) result in two device names,
e.g. for the Driver Station to recognize by name.

http://www.linux-usb.org/USB-guide/x75.html

https://www.engineersgarage.com/usb-descriptors-and-their-types-part-3-6/
