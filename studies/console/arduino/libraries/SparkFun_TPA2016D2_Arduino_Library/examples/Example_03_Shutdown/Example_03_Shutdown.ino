/******************************************************************************
  Example _03_Shutdown.ino
  Demonstrates how to shutdown the TPA2016D2 via software command.
  Enables and disables shutdown once every 5 seconds. "blinks" the sound :)

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
  Serial.println("Example 3 - Shutdown");

  Wire.begin();

  if (amp.begin() == false) //Begin communication over I2C
  {
    Serial.println("The device did not respond. Please check wiring.");
    while (1); //Freeze
  }
  Serial.println("Device is connected properly.");
}

void loop()
{
  amp.enableShutdown();
  Serial.println("Shutdown enabled.");
  delay(5000);

  amp.disableShutdown();
  Serial.println("ITS ALIVE!");
  delay(5000);
}