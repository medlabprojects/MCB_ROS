/*=========================================================================//

	AD5761R Class
	
	This class is used to configure and use the AD5761R 16-bit DAC over SPI
	
	Designed for use with Teensy 3.X and Motor Control Board (Rev 1.X)
	
	
	Trevor Bruns
	
	Changelog-
		2/10/16: Initial creation
		3/14/17: Changed names to conform to convention
				 Removed unused code

//=========================================================================*/

#include "AD5761R.h"
#include <SPI.h>
#include <stdint.h>

// default constructor
AD5761R::AD5761R(const uint8_t csDAC)
	: csDAC_(csDAC)
	, SPISettings_(4000000, MSBFIRST, SPI_MODE2)
{	
	// init chip select pin (SYNC)
	pinMode(csDAC, OUTPUT);
	digitalWriteFast(csDAC, HIGH);

	// ensure SPI has been started
	SPI.begin();

	// default settings
	uint32_t CV  = CV_ZERO;		// CLEAR voltage = zero scale 
	uint32_t OVR = OVR_DISABLE; // 5% overrange = disabled
	uint32_t B2C = B2C_COMP;	// Bipolar range = twos complement coded (signed)
	uint32_t ETS = ETS_ENABLE;  // Thermal shutdown = enabled
	uint32_t IRO = IRO_ON;		// Internal reference = on
	uint32_t PV  = PV_ZERO;		// Power up voltage = zero scale
	uint32_t RA  = RA_TEN_TEN;	// Output range = -10V to +10V
	//data_ctrl.value = (WR_CTRL << 16) | (CV << 9) | (OVR << 8) | (B2C << 7) | (ETS << 6) | (IRO << 5) | (PV << 3) | (RA);
	dataCtrl_.value = (WR_CTRL | CV | OVR | B2C | ETS | IRO | PV | RA);
}

void AD5761R::init(void)
{
	// !! NOTE: must use beginTransfer() and endTransfer() !!
	
	// set control register
	transfer(dataCtrl_);
}

void AD5761R::init(uint32_t dataCtrl) 
{
	// !! NOTE: must use beginTransfer() and endTransfer() !!

	// set control register
	dataCtrl_.value = dataCtrl;
	transfer(dataCtrl_);
}

void AD5761R::reset(void)
{
	// !! NOTE: must use beginTransfer() and endTransfer() !!

	uint8_uint32 cmd_reset;
	cmd_reset.value = SW_RESET_FULL;
	transfer(cmd_reset);
}

void AD5761R::set(int16_t output)
{
	// !! NOTE: must use beginTransfer() and endTransfer() !!

	uint8_uint32 temp;

	// signed 16 bit values are still stored in 32 bit register. Thus the signed bit is
	// bit 31 (MSB) rather than bit 15. This workaround corrects for this issue.
	temp.byte[0] = (uint8_t)(output & 0x00FF);
	temp.byte[1] = (uint8_t)((output >> 8) & 0x00FF);
	temp.byte[2] = WR_UPDATE>>16;
	transfer(temp);
}

void AD5761R::beginTransfer(void)
{
	SPI.beginTransaction(SPISettings_); // dummy transfer to ensure SPI settings have changed
	SPI.transfer(0x00);
	SPI.endTransaction(); 
	
	SPI.beginTransaction(SPISettings_);
	digitalWriteFast(csDAC_, LOW);
}

void AD5761R::endTransfer(void)
{
	digitalWriteFast(csDAC_, HIGH);
	SPI.endTransaction();
}

uint32_t AD5761R::transfer(uint8_uint32 dataOut)
{
	// !! NOTE: must call beginTransfer() first !!
	
	uint8_uint32 dataIn;	
	
	for(int8_t aa = 3 ; aa > 0 ; aa--) { // send 3 byte packet, MSB->LSB
		dataIn.byte[aa-1] = SPI.transfer(dataOut.byte[aa-1]);
	}	
	return dataIn.value;
}

AD5761R::~AD5761R(void)
{
	// reset DAC back to 0V output
	beginTransfer();
	reset();
	endTransfer();
}
