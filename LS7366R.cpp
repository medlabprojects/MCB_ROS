/*=========================================================================//

	LS7366R Class

	This class is used to configure and use the LS7366R Quadrature Coutner

	Designed for use with Teensy 3.X and Motor Control Board (Rev 1.X)


	Trevor Bruns

	Changelog-
	2/11/16: Initial creation
	3/14/17: Changed names to conform to convention
			 Removed unused code

//=========================================================================*/

#include "LS7366R.h"
#include "core_pins.h"
#include <SPI.h>

LS7366R::LS7366R(uint8_t csPin)
	:	csPin_(csPin)
	,	SPISettings_(3000000, MSBFIRST, SPI_MODE0)
{	
	// init chip-select pin (SYNC)
	 pinMode(csPin, OUTPUT);
	 digitalWriteFast(csPin, HIGH);
}

void LS7366R::init(void)
{
	// ensure SPI has been started
	SPI.begin();

	// Configure mode registers
	write(WRITE_MDR0, QUAD_4X | COUNT_FREE | INDEX_NO);

	write(WRITE_MDR1, SIZE_4 | CNT_ENABLE | NO_FLAGS);
	
	// Clear CNTR register
	write(CLR_CNTR);
}

uint8_t LS7366R::read(uint8_t opcode, uint8_t data_out)
{
	uint8_t data_in;
	
	SPI.beginTransaction(SPISettings_); // temporary fix when switching SPI modes
	SPI.transfer(0x00);					// sends a dummy byte to ensure correct mode is set
	SPI.endTransaction();
	
	SPI.beginTransaction(SPISettings_);
	digitalWriteFast(csPin_, LOW);
	SPI.transfer(opcode);
	data_in = SPI.transfer(data_out);
	digitalWriteFast(csPin_, HIGH);
	SPI.endTransaction();
	
	return data_in;
}

void LS7366R::write(uint8_t opcode, uint8_t data_out)
{
	SPI.beginTransaction(SPISettings_); // temporary fix when switching SPI modes
	SPI.transfer(0x00);					// sends a dummy byte to ensure correct mode is set
	SPI.endTransaction();
	
	SPI.beginTransaction(SPISettings_);
	digitalWriteFast(csPin_, LOW);
	SPI.transfer(opcode);
	SPI.transfer(data_out);
	digitalWriteFast(csPin_, HIGH);
	SPI.endTransaction();
}

void LS7366R::write(uint8_t opcode)
{
	SPI.beginTransaction(SPISettings_); // temporary fix when switching SPI modes
	SPI.transfer(0x00);					// sends a dummy byte to ensure correct mode is set
	SPI.endTransaction();
	
	SPI.beginTransaction(SPISettings_);
	digitalWriteFast(csPin_, LOW);
	SPI.transfer(opcode);
	digitalWriteFast(csPin_, HIGH);
	SPI.endTransaction();
}

int32_t LS7366R::count(void)
{

	uint8_uint32 count_temp;
	
	SPI.beginTransaction(SPISettings_); // temporary fix when switching SPI modes
	SPI.transfer(0x00);					// sends a dummy byte to ensure correct mode is set
	SPI.endTransaction();
	
	SPI.beginTransaction(SPISettings_);
	digitalWriteFast(csPin_, LOW);
	SPI.transfer(READ_CNTR);
	for (int8_t aa = 3; aa > -1; aa--) { // read out each byte, MSB->LSB
		count_temp.byte[aa] = SPI.transfer(0x00);
	}
	digitalWrite(csPin_, HIGH);
	SPI.endTransaction();
	
	return (int32_t) count_temp.value;
}

uint8_t LS7366R::status(void)
{
	return read(READ_STR, 0x00);
}

LS7366R::~LS7366R(void)
{
}
