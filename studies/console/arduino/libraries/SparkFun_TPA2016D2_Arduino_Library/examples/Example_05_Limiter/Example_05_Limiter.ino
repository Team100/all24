/******************************************************************************
  Example _05_Limiter.ino
  Allows the user to set the limiter threshold value using the Serial Monitor (115200).

  Note, this example also sets the following settings on the amp.
  These settings are pretty good for most audio sources,
  and they also make the limiter adjustments more noticable to the listener.
  Try the more advanced examples to adjust other settings.
  gain: 30dB (MAX).
  compression ratio: 8:1 (most aggressive)
  attack: 1 (fastest)
  release: 1 (fastest)

  SparkFun TPA2016D2 Arduino Library
  Pete Lewis @ SparkFun Electronics
  September 8, 2022
  https://github.com/sparkfun/SparkFun_TPA2016D2_Arduino_Library

  This code was originally created by Mike Grusin at SparkFun Electronics
  Included with the LilyPad MP3 example code found here:
  Revision history: version 1.0 2012/07/24 MDG Initial release
  https://github.com/sparkfun/LilyPad_MP3_Player

  Do you like this library? Help support SparkFun. Buy a board!

    SparkFun Qwiic Speaker Amp - TPA2016D2
    https://www.sparkfun.com/products/20690

  Development environment specifics:

  IDE: Arduino 1.8.19
  Hardware Platform: SparkFun Redboard Qwiic
  SparkFun Qwiic Speaker Amp - TPA2016D2 Version: 1.0

  Hardware Connections:
  Use a qwiic cable to connect from the Redboard Qwiic to the Qwiic Speaker Amp.
  Connect audio-in, speakers, and power to the Qwiic Speaker Amp.

  For information on the data sent to and received from the amplifier,
  refer to the TPA2016D2 datasheet at:
  http://www.ti.com/lit/ds/symlink/tpa2016d2.pdf

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#include <Wire.h>
#include <SparkFun_TPA2016D2_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_TPA2016D2
TPA2016D2 amp;

unsigned char limiterThreshold = 31;    // (0-31 are valid), let's start high, so the limiter is effectively "off".
// lower thresholds will "squash" the output and keep things quieter.

void setup()
{
  Serial.begin(115200);
  Serial.println("Example 5 - Setting Limiter Values.");

  Wire.begin();

  if (amp.begin() == false) //Begin communication over I2C
  {
    Serial.println("The device did not respond. Please check wiring.");
    while (1); //Freeze
  }
  Serial.println("Device is connected properly.");

  // for limiter threshold control to be more noticable by the listener,
  // we need to configure the amp with certain AGC settings...
  configure_amp();

  print_serial_menu();
}

void loop()
{
  while (Serial.available() > 0)
  {
    limiterThreshold = Serial.parseInt();
    configure_amp(); // updates limiter threshold and writes some other settings too!
    print_serial_menu();
  }
}

void print_serial_menu()
{
  Serial.println("Qwiic Speaker Amp Example 5 - Limiter Threshold");
  Serial.print("Limiter Threshold:");
  Serial.println(limiterThreshold, DEC);
  Serial.println("To set a new limiter threshold, type a number (0-31) into the serial monitor");
}

void configure_amp()
{
  amp.enableLimiter();
  amp.writeOutputLimiterLevel(limiterThreshold); // update to current/new value.
  amp.writeCompressionRatio(COMPRESSION_RATIO_8_1); // aggressive ratio, so it really "squashes" everything above the threshold.
  amp.disableNoiseGate(); // disabling the noisegate allows us to always change the gain, even with very little sound at the source.
  amp.writeRelease(1); // 1-63 are valid values. 1 being the shortest (aka fastest) release setting, this allows gain increases to happen quickly.
  amp.writeAttack(1); // 1-63 are valid values. 1 being the shortest (aka fastest) attack setting, this allows gain decreases to happen quickly.
  amp.writeFixedGain(30); // aka "full gain at +30dB", accepts values from 0 to 30
}