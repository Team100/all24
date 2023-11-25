# Console

Java and Arduino code for FRC operator console.

The Arduino code emulates a USB Joystick device, with up to
32 buttons, 8 motion axes, and 16 LED outputs.

To build the Arduino part, you use the [Arduino IDE](https://www.arduino.cc/en/software).
Open one of the console files, e.g. knobs.ino.
Choose "preferences" and set the "sketchbook location" (the first line in the preferences dialog box)
to all24/studies/console/arduino, to pick up the required libraries.

To program the Sparkfun Pro Micro, you have to push the little button, so make it easy to access.

There's an example Fusion 360 assembly for laser-cut panels [here.](https://a360.co/3Fu8oDa)
If anybody knows how to share the entire Fusion 360 project, please let me know.

Absolute encoder and joystick for swerve pilot, also trims:

<img src="pilot.jpg" width=500/>

Many buttons for everything else:

<img src="buttons.jpg" width=500/>

Audio output for target-lock feedback, also buttons:

<img src="autopilot.jpg" width=500/>
