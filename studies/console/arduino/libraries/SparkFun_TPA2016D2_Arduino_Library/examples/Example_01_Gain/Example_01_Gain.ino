/******************************************************************************
  Example _01_Gain.ino
  Sets a few different gain values on the TPA2016D2 speaker amp.

  Note, when gain is "0", it still passes sound through.
  To turn the sound off, use shutdown or enable/disable examples.

  Note, you can't REALLY turn off the AGC on the TPA2016D2,
  But if you disable the limiter, noisegate, and set fast release/attack
  times, then it only minimally effects gain changes.

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

void setup()
{
  Serial.begin(115200);
  Serial.println("Example 1 - Setting Gain Values.");

  Wire.begin();

  if (amp.begin() == false) //Begin communication over I2C
  {
    Serial.println("The device did not respond. Please check wiring.");
    while (1); //Freeze
  }
  Serial.println("Device is connected properly.");

  // for gain control to react to changes quickly, we need to adjust some of the AGC settings as so...
  amp.disableLimiter(); // note this also changes compression ratio to 1:1, then disables limiter.
  amp.disableNoiseGate(); // disabling the noisegate allows us to always change the gain, even with very little sound at the source.
  amp.writeRelease(1); // 1-63 are valid values. 1 being the shortest (aka fastest) release setting, this allows gain increases to happen quickly.
  amp.writeAttack(1); // 1-63 are valid values. 1 being the shortest (aka fastest) attack setting, this allows gain decreases to happen quickly.

  Serial.println("gain:+30 (max)");
  amp.writeFixedGain(30); // aka "full gain at +30dB", accepts values from 0 to 30
  delay(5000);

  Serial.println("gain:+15 (mid)");
  amp.writeFixedGain(15);
  delay(5000);

  Serial.println("gain:0 (min)");
  amp.writeFixedGain(0);
  delay(5000);

  Serial.println("Example complete. Hit Reset to try again.");
}

void loop()
{
  // nothing to see here.
}