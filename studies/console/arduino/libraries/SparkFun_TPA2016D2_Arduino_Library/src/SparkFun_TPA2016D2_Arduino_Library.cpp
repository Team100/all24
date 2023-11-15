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

#include <SparkFun_TPA2016D2_Arduino_Library.h>

TPA2016D2::TPA2016D2() {
  // constructor does nothing, must call Wire.begin() from within setup()
}

boolean TPA2016D2::begin(TwoWire &wirePort)
{
    //_deviceAddress = _deviceAddress;
  	_i2cPort = &wirePort;
  	if (isConnected() == false) // Check for sensor by verifying ACK response
    	return (false); 
  	return (true); //We're all setup!
}

//Returns true if I2C device ack's
boolean TPA2016D2::isConnected()
{
  	_i2cPort->beginTransmission((uint8_t)_deviceAddress);
  	if (_i2cPort->endTransmission() != 0)
    	return (false); //Sensor did not ACK
  	return (true);
}

unsigned char TPA2016D2::writeRegister(unsigned char reg, unsigned char value)
// General-purpose write to a TPA2016D2 register
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char result;

  _i2cPort->beginTransmission((uint8_t)_deviceAddress);// I2C address (use 7-bit address, wire library will modify for read/write)
  _i2cPort->write(reg);                       // register to write
  _i2cPort->write(value);                     // value to write
  result = _i2cPort->endTransmission();

  if (result == 0)
     return 1;
  return 0;
}


unsigned char TPA2016D2::readRegister(unsigned char reg, unsigned char *value)
// General-purpose read from a TPA2016D2 register
// Call with the address of your value variable e.g. &value
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char result, x;

  _i2cPort->beginTransmission((uint8_t)_deviceAddress);// i2c address (use 7-bit address, wire library will modify for read/write)
  _i2cPort->write(reg);                       // register to read
  result = _i2cPort->endTransmission();

  if (result == 0) // successful setup
  {
    _i2cPort->requestFrom((uint8_t)_deviceAddress, 1); 
    x = _i2cPort->available(); // make sure read was successful
  
    if (x == 1)
    {
        *value = _i2cPort->read();
        return 1;
    }
  }
  return 0;
}

unsigned char TPA2016D2::enableSpeakers()
// Enable both output channels (unmute)
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register, modify only the bits we need, and write back the modified value
  if (TPA2016D2::readRegister(TPA2016D2_CONTROL_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_CONTROL_REGISTER,((regvalue | B11000000))));
      return 1;
  }
  return 0;
}

unsigned char TPA2016D2::disableSpeakers()
// Disable both output channels (mute)
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register, modify only the bits we need, and write back the modified value
  if (TPA2016D2::readRegister(TPA2016D2_CONTROL_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_CONTROL_REGISTER,((regvalue & B00111111))));
      return 1;
  }
  return 0;
}

unsigned char TPA2016D2::enableRightSpeaker()
// Enable the right output channel
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register, modify only the bits we need, and write back the modified value
  if (TPA2016D2::readRegister(TPA2016D2_CONTROL_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_CONTROL_REGISTER,((regvalue | B10000000))));
      return 1;
  }
  return 0;
}


unsigned char TPA2016D2::disableRightSpeaker()
// Disable the right output channel
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register, modify only the bits we need, and write back the modified value
  if (TPA2016D2::readRegister(TPA2016D2_CONTROL_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_CONTROL_REGISTER,((regvalue & B01111111))));
      return 1;
  }
  return 0;
}


unsigned char TPA2016D2::enableLeftSpeaker()
// Enable the left output channel
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register, modify only the bits we need, and write back the modified value
  if (TPA2016D2::readRegister(TPA2016D2_CONTROL_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_CONTROL_REGISTER,((regvalue | B01000000))));
      return 1;
  }
  return 0;
}


unsigned char TPA2016D2::disableLeftSpeaker()
// Disable the left output channel
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register, modify only the bits we need, and write back the modified value
  if (TPA2016D2::readRegister(TPA2016D2_CONTROL_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_CONTROL_REGISTER,((regvalue & B10111111))));
      return 1;
  }
  return 0;
}


unsigned char TPA2016D2::enableShutdown()
// Shut down the amplifier (overrides external input)
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register, modify only the bits we need, and write back the modified value
  if (TPA2016D2::readRegister(TPA2016D2_CONTROL_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_CONTROL_REGISTER,((regvalue | B00100000))));
      return 1;
  }
  return 0;
}


unsigned char TPA2016D2::disableShutdown()
// Enable the amplifier (external input will override this setting)
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register, modify only the bits we need, and write back the modified value
  if (TPA2016D2::readRegister(TPA2016D2_CONTROL_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_CONTROL_REGISTER,((regvalue & B11011111))));
      return 1;
  }
  return 0;
}

boolean TPA2016D2::readFaultLeft()
{
  TPA2016D2::readFaults(&_leftFault, &_rightFault, &_thermalFault);
  if(_leftFault) return 1;
  return 0;
}

boolean TPA2016D2::readFaultRight()
{
  TPA2016D2::readFaults(&_leftFault, &_rightFault, &_thermalFault);
  if(_rightFault) return 1;
  return 0;
}

boolean TPA2016D2::readFaultThermal()
{
  TPA2016D2::readFaults(&_leftFault, &_rightFault, &_thermalFault);
  if(_thermalFault) return 1;
  return 0;
}

unsigned char TPA2016D2::readFaults(unsigned char *left, unsigned char *right, unsigned char *thermal)
// Return all three fault settings.
// Call with addresses of your variables e.g. &left
// Your variable will be 1 if there is a fault, 0 otherwise.
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register and extract the bits we need
  if (TPA2016D2::readRegister(TPA2016D2_CONTROL_REGISTER,&regvalue))
  {
    *right   = ((regvalue & B00010000) >> 4);	// mask and shift results
    *left    = ((regvalue & B00001000) >> 3);
    *thermal = ((regvalue & B00000100) >> 2);
    return 1;
  }
  else
    return 0;
}

unsigned char TPA2016D2::resetFaults()
// reset all fault flags 
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register, modify only the bits we need, and write back the modified value
  if (TPA2016D2::readRegister(TPA2016D2_CONTROL_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_CONTROL_REGISTER,((regvalue & B11100011)))); //Clear bits 2,3,4
      return 1;
  }
  return 0;
}


unsigned char TPA2016D2::enableNoiseGate()
// Enable the noise gate feature
// Note that the noise gate can only be enabled if the compression ratio is not 1:1
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register, modify only the bits we need, and write back the modified value
  if (TPA2016D2::readRegister(TPA2016D2_CONTROL_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_CONTROL_REGISTER,((regvalue | B00000001))));
      return 1;
  }
  return 0;
}


unsigned char TPA2016D2::disableNoiseGate()
// Disable the noise gate feature
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register, modify only the bits we need, and write back the modified value
  if (TPA2016D2::readRegister(TPA2016D2_CONTROL_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_CONTROL_REGISTER,((regvalue & B11111110))));
      return 1;
  }
  return 0;
}


unsigned char TPA2016D2::writeAttack(unsigned char attack)
// Write the AGC attack time (0-63, see datasheet for units)
// Returns 1 if successful, 0 if something failed (I2C error)
{
  if (TPA2016D2::writeRegister(TPA2016D2_ATTACK_REGISTER,(attack & B00111111)))
    return 1;
  else
    return 0;
}


unsigned char TPA2016D2::readAttack(unsigned char *attack)
// Read the AGC attack time (0-63, see datasheet for units)
// Call with address of your variable e.g. &attack
// Returns 1 if successful, 0 if something failed (I2C error)
{
  if (TPA2016D2::readRegister(TPA2016D2_ATTACK_REGISTER,attack))
    return 1;
  else
    return 0;
}


unsigned char TPA2016D2::writeRelease(unsigned char release)
// Write the AGC release time (0-63, see datasheet for units)
// Returns 1 if successful, 0 if something failed (I2C error)
{
  if (TPA2016D2::writeRegister(TPA2016D2_RELEASE_REGISTER,(release & B00111111)))
    return 1;
  else
    return 0;
}


unsigned char TPA2016D2::readRelease(unsigned char *release)
// Read the AGC release time (0-63, see datasheet for units)
// Call with address of your variable e.g. &release
// Returns 1 if successful, 0 if something failed (I2C error)
{
  if (TPA2016D2::readRegister(TPA2016D2_RELEASE_REGISTER,release))
    return 1;
  else
    return 0;
}


unsigned char TPA2016D2::writeHold(unsigned char hold)
// Write the AGC hold time (0-63, see datasheet for units)
// Returns 1 if successful, 0 if something failed (I2C error)
{
  if (TPA2016D2::writeRegister(TPA2016D2_HOLD_REGISTER,(hold & B00111111)))
    return 1;
  else
    return 0;
}


unsigned char TPA2016D2::readHold(unsigned char *hold)
// Read the AGC hold time (0-63, see datasheet for units)
// Call with address of your variable e.g. &hold
// Returns 1 if successful, 0 if something failed (I2C error)
{
  if (TPA2016D2::readRegister(TPA2016D2_HOLD_REGISTER,hold))
    return 1;
  else
    return 0;
}


unsigned char TPA2016D2::writeFixedGain(char gain)
// Write the AGC fixed gain (-28 to +30, see datasheet for units)
// Returns 1 if successful, 0 if something failed (I2C error)
{
  if (TPA2016D2::writeRegister(TPA2016D2_GAIN_REGISTER,(gain & B00111111)))
    return 1;
  else
    return 0;
}


unsigned char TPA2016D2::readFixedGain(char *gain)
// Read the AGC fixed gain (-28 to +30, see datasheet for units)
// Call with address of your variable e.g. &gain
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

  if (TPA2016D2::readRegister(TPA2016D2_GAIN_REGISTER,&regvalue))
  {
    if (regvalue & B00100000) // negative number?
    {
      regvalue |= B11000000; // sign-extend, so that it stays negative
    }
    *gain = (char)regvalue;
    return 1;
  }
  return 0;
}


unsigned char TPA2016D2::disableLimiter()
// Set the compression ratio to 1:1 (required) and disable the output limiter
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the registers, modify only the bits we need, and write back the modified values
  if (TPA2016D2::readRegister(TPA2016D2_AGC2_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_AGC2_REGISTER,(regvalue & B11111100))) // set compression ratio to 0
    {
      if (TPA2016D2::readRegister(TPA2016D2_AGC1_REGISTER,&regvalue))
      {
        if (TPA2016D2::writeRegister(TPA2016D2_AGC1_REGISTER,(regvalue | B10000000))) // disable limiter
          return 1;
      }
    }
  }
  return 0;
}


unsigned char TPA2016D2::enableLimiter()
// Enable the output limiter
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register, modify only the bits we need, and write back the modified value
  if (TPA2016D2::readRegister(TPA2016D2_AGC1_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_AGC1_REGISTER,(regvalue & B01111111))) // enable limiter
      return 1;
  }
  return 0;
}


unsigned char TPA2016D2::writeNoiseGateThreshold(unsigned char noisegatethreshold)
// Write AGC noise gate threshold (0-3, see datasheet for units)
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register, modify only the bits we need, and write back the modified value
  if (TPA2016D2::readRegister(TPA2016D2_AGC1_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_AGC1_REGISTER,((regvalue & B10011111) | ((noisegatethreshold << 5) & B01100000))));
      return 1;
  }
  return 0;
}


unsigned char TPA2016D2::readNoiseGateThreshold(unsigned char *noisegatethreshold)
// Read AGC noise gate threshold (0-3, see datasheet for units)
// Call with address of your variable e.g. &noisegatethreshold
// Returns 1 if successful, 0 if something failed (I2C error)
{
  if (TPA2016D2::readRegister(TPA2016D2_AGC1_REGISTER,noisegatethreshold))
  {
    *noisegatethreshold = ((*noisegatethreshold >> 5) & B00000011);	// shift and mask result
    return 1;
  }
  return 0;
}


unsigned char TPA2016D2::writeOutputLimiterLevel(unsigned char outputlimiterlevel)
// Write AGC output limiter level (0-31, see datasheet for units)
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register, modify only the bits we need, and write back the modified value
  if (TPA2016D2::readRegister(TPA2016D2_AGC1_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_AGC1_REGISTER,((regvalue & B11100000) | (outputlimiterlevel & B00011111))));
      return 1;
  }
  return 0;
}


unsigned char TPA2016D2::readOutputLimiterLevel(unsigned char *outputlimiterlevel)
// Read AGC output limiter level (0-31, see datasheet for units)
// Call with address of your variable e.g. &outputlimiterlevel
// Returns 1 if successful, 0 if something failed (I2C error)
{
  if (TPA2016D2::readRegister(TPA2016D2_AGC1_REGISTER,outputlimiterlevel))
  {
    *outputlimiterlevel &= B00011111;	// mask result
    return 1;
  }
  return 0;
}


unsigned char TPA2016D2::writeMaxGain(unsigned char maxgain)
// Write AGC max gain (0-15, see datasheet for units)
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register, modify only the bits we need, and write back the modified value
  if (TPA2016D2::readRegister(TPA2016D2_AGC2_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_AGC2_REGISTER,((regvalue & B00001111) | ((maxgain << 4) & B11110000))));
      return 1;
  }
  return 0;
}


unsigned char TPA2016D2::readMaxGain(unsigned char *maxgain)
// Read AGC max gain (0-15, see datasheet for units)
// Call with address of your variable e.g. &maxgain
// Returns 1 if successful, 0 if something failed (I2C error)
{
  if (TPA2016D2::readRegister(TPA2016D2_AGC2_REGISTER,maxgain))
  {
    *maxgain = ((*maxgain >> 4) & B00001111);	// shift and mask result
    return 1;
  }
  return 0;
}


unsigned char TPA2016D2::writeCompressionRatio(unsigned char compressionratio)
// Write AGC compression ratio (0-3, see datasheet for units)
// Returns 1 if successful, 0 if something failed (I2C error)
{
  unsigned char regvalue;

	// Read the register, modify only the bits we need, and write back the modified value
  if (TPA2016D2::readRegister(TPA2016D2_AGC2_REGISTER,&regvalue))
  {
    if (TPA2016D2::writeRegister(TPA2016D2_AGC2_REGISTER,((regvalue & B11111100) | (compressionratio & B00000011))));
      return 1;
  }
  return 0;
}


unsigned char TPA2016D2::readCompressionRatio(unsigned char *compressionratio)
// Read AGC compression ratio (0-3, see datasheet for units)
// Call with address of your variable e.g. &compressionratio
// Returns 1 if successful, 0 if something failed (I2C error)
{
  if (TPA2016D2::readRegister(TPA2016D2_AGC2_REGISTER,compressionratio))
  {
    *compressionratio &= B00000011;	// mask result
    return 1;
  }
  return 0;
}

