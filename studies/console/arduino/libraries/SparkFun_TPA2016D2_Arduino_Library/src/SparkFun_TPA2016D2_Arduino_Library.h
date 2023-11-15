/******************************************************************************
  SparkFun TPA2016D2 Arduino Library

  This library provides a set of functions to control the TI TPA2016D2
	stereo amplifier via I2C.

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
	
	All functions return 1 if the read/write was successful, and 0
	if there was a communications failure. You can ignore the return value
	if you just don't care anymore.
	
	For functions that return a value, e.g. "readGain(char *gain)", first
	declare	a variable of the proper type e.g. "char x", then pass the
	address of your variable to the function by putting '&' in front of it
	e.g. "readGain(&x)". The function will modify the variable directly.

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

#ifndef __SparkFunTPA2016D2_H__
#define __SparkFunTPA2016D2_H__

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

// I2C address (7-bit format for Wire library)
#define TPA2016D2_ADDR 0x58

// TPA2016D2 registers
#define TPA2016D2_CONTROL_REGISTER 1
#define TPA2016D2_ATTACK_REGISTER 2
#define TPA2016D2_RELEASE_REGISTER 3
#define TPA2016D2_HOLD_REGISTER 4
#define TPA2016D2_GAIN_REGISTER 5
#define TPA2016D2_AGC1_REGISTER 6
#define TPA2016D2_AGC2_REGISTER 7

// control settings
#define COMPRESSION_RATIO_1_1 0x00
#define COMPRESSION_RATIO_2_1 0x01
#define COMPRESSION_RATIO_4_1 0x10
#define COMPRESSION_RATIO_8_1 0x11


class TPA2016D2
{
	public:
	
		TPA2016D2();

		boolean begin(TwoWire &wirePort = Wire);

		boolean isConnected();

		// Enable and disable speakers (mute)
		unsigned char enableSpeakers();
		unsigned char disableSpeakers();
		unsigned char enableRightSpeaker();
		unsigned char disableRightSpeaker();
		unsigned char enableLeftSpeaker();
		unsigned char disableLeftSpeaker();
		
		// Shut down the amplifier
		unsigned char enableShutdown();
		unsigned char disableShutdown();
		
		// Read fault conditions
		unsigned char readFaults(unsigned char *left, unsigned char *right, unsigned char *thermal);
		boolean readFaultLeft();
		boolean readFaultRight();
		boolean readFaultThermal();
		unsigned char resetFaults();

		
		// Enable and disable noise gate
		unsigned char enableNoiseGate();
		unsigned char disableNoiseGate();
		
		// Read/write AGC attack time (0-63, see datasheet for units)
		unsigned char writeAttack(unsigned char attack);
		unsigned char readAttack(unsigned char *attack);
		
		// Read/write AGC release time (0-63, see datasheet for units)
		unsigned char writeRelease(unsigned char release);
		unsigned char readRelease(unsigned char *release);
		
		// Read/write AGC hold time (0-63, see datasheet for units)
		unsigned char writeHold(unsigned char hold);
		unsigned char readHold(unsigned char *hold);
		
		// Read/write AGC fixed gain (-28 to +30, see datasheet for units)
		unsigned char writeFixedGain(char gain);
		unsigned char readFixedGain(char *gain);
		
		// Enable and disable limiter
		unsigned char disableLimiter();
		unsigned char enableLimiter();
		
		// Read/write AGC noise gate threshold (0-3, see datasheet for units)
		unsigned char writeNoiseGateThreshold(unsigned char noisegatethreshold);
		unsigned char readNoiseGateThreshold(unsigned char *noisegatethreshold);
		
		// Read/write AGC output limiter level (0-31, see datasheet for units)
		unsigned char writeOutputLimiterLevel(unsigned char outputlimiterlevel);
		unsigned char readOutputLimiterLevel(unsigned char *outputlimiterlevel);

		// Read/write AGC max gain (0-15, see datasheet for units)
		unsigned char writeMaxGain(unsigned char maxgain);
		unsigned char readMaxGain(unsigned char *maxgain);

		// Read/write AGC compression ratio (0-3, see datasheet for units)
		unsigned char writeCompressionRatio(unsigned char compressionratio);
		unsigned char readCompressionRatio(unsigned char *compressionratio);

		// General-purpose register read/write
		unsigned char writeRegister(unsigned char reg, unsigned char value);
		unsigned char readRegister(unsigned char reg, unsigned char *value);

	private:
		TwoWire *_i2cPort;
		uint8_t _deviceAddress = TPA2016D2_ADDR;
		unsigned char _leftFault = 0;
		unsigned char _rightFault = 0;
		unsigned char _thermalFault = 0;
};




#endif
