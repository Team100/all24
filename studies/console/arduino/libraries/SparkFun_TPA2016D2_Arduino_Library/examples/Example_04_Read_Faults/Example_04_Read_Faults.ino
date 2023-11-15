/******************************************************************************
  Example _04_Read_Faults.ino
  Reads the faults from the TPA2016D2 and prints them to terminal, once a second.

  DO NOT connect speakers leads while sound is playing through speaker.

  This should only be used to detect shorts on speaker lines BEFORE actually turning on your source.

  If you connect the "+" and "-" terminals of either speaker, then the TPA2016D2
  can detect this "fault condition". and protect itself.
  Load this sketch, and watch the terminal to see the faults trigger when you 
  connect "+" to "-".
  After 3 seconds, it will reset the fault register flags, reset the device (via shutdown) 
  and start printing statuses again.

  Note, in order to cause a Thermal Fault, you may need a louder audio source, a higher gain
  settings, and/or the limiting disabled.

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

boolean faultRight, faultLeft, faultThermal;

void setup()
{
  Serial.begin(115200);
  Serial.println("Example 4 - Read Faults");

  Wire.begin();

  if (amp.begin() == false) //Begin communication over I2C
  {
    Serial.println("The device did not respond. Please check wiring.");
    while(1); //Freeze
  }
  Serial.println("Device is connected properly.");
}

void loop()
{
  faultLeft = amp.readFaultLeft();
  faultRight = amp.readFaultRight();
  faultThermal = amp.readFaultThermal();

  if (faultLeft == true) Serial.println("Left speaker fault detected!");
  if (faultRight == true) Serial.println("Right speaker fault detected!");
  if (faultThermal == true) Serial.println("Thermal fault detected!");

  if(faultLeft || faultRight || faultThermal)
  {
    Serial.println("Waiting 3 seconds, then reseting faults...");
    delay(3000);
    amp.resetFaults();
    delay(1000);
    amp.enableShutdown();
    delay(1000);
    amp.disableShutdown();
  }
  else
  {
    Serial.println("No faults detected :)");
    delay(1000);
  }
}