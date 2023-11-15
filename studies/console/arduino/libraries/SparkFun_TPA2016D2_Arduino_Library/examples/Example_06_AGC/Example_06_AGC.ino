/******************************************************************************
  Example _06_AGC.ino
  Allows the user to set the AGC values using the Serial Monitor (115200).

  Note, this example also sets the following settings on the amp on bootup.
  These settings are pretty good for most audio sources,
  and they also make the limiter adjustments more noticable to the listener.
  Using the Serial Monitor, you can adjust many of the settings.
  Note, these settings do not retain in the TPA2016D2. They will be reset to defaults
  after each power cycle.
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

char fixedGain = 30;
unsigned char limiterThreshold = 31;    // (0-31 are valid), let's start high, so the limiter is effectively "off".
// lower thresholds will "squash" the output and keep things quieter.
unsigned char compressionRatio = COMPRESSION_RATIO_8_1;
unsigned char attack = 1;
unsigned char release = 1;
unsigned char hold = 0;
unsigned char user_input_param = 0;
char user_input_val = 0;


void setup()
{
  Serial.begin(115200);
  Serial.println("Example 6 - Setting AGC Values.");

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
    user_input_param = Serial.parseInt();
    //Serial.print("user_input_param:");
    //Serial.println(user_input_param);

    Serial.print("You have selected the paramter: ");
    if (user_input_param == 1) Serial.println("Gain");
    if (user_input_param == 2) Serial.println("Limiter Threshold");
    if (user_input_param == 3) Serial.println("Compression Ratio");
    if (user_input_param == 4) Serial.println("Attack");
    if (user_input_param == 5) Serial.println("Release");
    if (user_input_param == 6) Serial.println("Hold");

    Serial.println("Please type in a new value...");

    listen_for_val();
    update_val();

    configure_amp(); // updates all settings.
    print_serial_menu();
  }
}

void print_serial_menu()
{
  Serial.println();
  Serial.println("Qwiic Speaker Amp Example 6 - AGC settings");
  Serial.println("Type a number to select a parameter you would like to change.");
  Serial.println("# - parameter: current setting [valid input]");

  Serial.print("1 - Gain: ");
  Serial.print(fixedGain, DEC);
  Serial.println(" [-28 to +30]");

  Serial.print("2 - Limiter Threshold: ");
  Serial.print(limiterThreshold, DEC);
  Serial.println(" [0-31]");

  Serial.print("3 - Compression Ratio: ");
  switch (compressionRatio)
  {
    case COMPRESSION_RATIO_8_1:
      Serial.print("8:1");
      break;
    case COMPRESSION_RATIO_4_1:
      Serial.print("4:1");
      break;
    case COMPRESSION_RATIO_2_1:
      Serial.print("2:1");
      break;
    case COMPRESSION_RATIO_1_1:
      Serial.print("1:1");
      break;
  }
  Serial.println(" [1,2,4,8]");

  Serial.print("4 - Attack: ");
  Serial.print(attack, DEC);
  Serial.println(" [1-63]");

  Serial.print("5 - Release: ");
  Serial.print(release, DEC);
  Serial.println(" [1-63]");

  Serial.print("6 - Hold: ");
  Serial.print(hold, DEC);
  Serial.println(" [1-63]");

  Serial.println();
}

void configure_amp()
{
  amp.enableLimiter();
  amp.writeOutputLimiterLevel(limiterThreshold); // update to current/new value.
  amp.writeCompressionRatio(compressionRatio); // aggressive ratio, so it really "squashes" everything above the threshold.
  amp.disableNoiseGate(); // disabling the noisegate allows us to always change the gain, even with very little sound at the source.
  amp.writeRelease(release); // 1-63 are valid values. 1 being the shortest (aka fastest) release setting, this allows gain increases to happen quickly.
  amp.writeAttack(attack); // 1-63 are valid values. 1 being the shortest (aka fastest) attack setting, this allows gain decreases to happen quickly.
  amp.writeFixedGain(fixedGain); // aka "full gain at +30dB", accepts values from 0 to 30
  amp.writeHold(hold); // default is "0" so effectively "off". Hold is the minimum time between a gain decrease (attack) and a gain increase (release).
}

void listen_for_val()
{
  boolean new_value_arrived = false;
  while (new_value_arrived == false)
  {
    while (Serial.available() > 0)
    {
      user_input_val = Serial.parseInt();
      new_value_arrived = true;
    }
  }
  //Serial.print("user_input_val:");
  //Serial.println(user_input_val);
}

void update_val()
{
  switch (user_input_param)
  {
    case 1:
      fixedGain = user_input_val;
      break;
    case 2:
      limiterThreshold = user_input_val;
      break;
    case 3:
      if (user_input_val == 1) compressionRatio = COMPRESSION_RATIO_1_1;
      if (user_input_val == 2) compressionRatio = COMPRESSION_RATIO_2_1;
      if (user_input_val == 4) compressionRatio = COMPRESSION_RATIO_4_1;
      if (user_input_val == 8) compressionRatio = COMPRESSION_RATIO_8_1;
      break;
    case 4:
      attack = user_input_val;
      break;
    case 5:
      release = user_input_val;
      break;
    case 6:
      hold = user_input_val;
      break;
  }
}