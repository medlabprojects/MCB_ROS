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


#ifndef AD5761R_h
#define AD5761R_h

#include <SPI.h>
#include <stdint.h>

// Register Addresses
#define NOP				0x000000	// No operation
#define	WR_INPUT_NOUP	0x010000	// Write to input register (no update)
#define UPDATE			0x020000	// Update DAC register from input register
#define WR_UPDATE		0x030000	// Write and update DAC register
#define	WR_CTRL			0x040000	// Write to control register
#define SW_RESET		0x070000	// Software data reset
#define	DISABLE_DAISY	0x090000	// Disable daisy-chain functionality
#define R_INPUT			0x0A0000	// Readback input register
#define R_DAC			0x0B0000	// Readback DAC register
#define R_CTRL			0x0C0000	// Readback control register
#define	SW_RESET_FULL	0x0F0000	// Software full reset

// Control Register Functions
// CV- clear voltage selection
#define CV_ZERO	0x000000	// zero scale
#define CV_MID	0x000400	// midscale
#define CV_FULL	0x000800	// full scale
// OVR- 5% overrange
#define OVR_DISABLE	0x000000	// 5% overrange disabled
#define OVR_ENABLE	0x000100	// 5% overrange enabled
// B2C- bipolar range
#define B2C_BIN		0x000000	// DAC input for bipolar output is straight binary coded
#define B2C_COMP	0x000080	// DAC input for bipolar output range is twos complement coded
// ETS- thermal shutdown alert
#define ETS_DISABLE 0x000000	// Internal digital supply does not power down if die temp > 150C
#define ETS_ENABLE	0x000040	// Internal digital supply powers down if die temp > 150C
// IRO- internal reference
#define IRO_OFF		0x000000	// Internal reference off
#define IRO_ON		0x000020	// Internal reference on
// PV- power up voltage
#define PV_ZERO	0x000000	// zero scale
#define PV_MID	0x000008	// midscale
#define PV_FULL	0x000010	// full scale
// RA- output range. Note: after an output range configuration, the device must be reset
#define RA_TEN_TEN		0x000000	//  -10V to +10V
#define RA_ZERO_TEN		0x000001	//    0V to +10V
#define RA_FIVE_FIVE	0x000002	//   -5V to +5V
#define RA_ZERO_FIVE	0x000003	//    0V to +5V
#define RA_TWO_SEVEN	0x000004	// -2.5V to +7.5V
#define RA_THREE_THREE  0x000005	//   -3V to +3V
#define RA_ZERO_SIXTEEN 0x000006	//	  0V to +16V
#define RA_ZERO_TWENTY  0x000007	//	  0V to +20V

#ifndef LS7366R_h
	union uint8_uint32
	{
		uint8_t byte[4];
		uint32_t value;
	};			  
#endif // !LS7366R_h



class AD5761R
{
public:
	AD5761R(const uint8_t csDAC); // construct with chip-select pin
	void init(void);  // initializes using default settings
	void init(uint32_t dataCtrl); // initialized using custom settings (|OR| your chosen control register #defines)
	void reset(void); // software reset (required after changing output range)
	void set(int16_t output); // set the DAC output

	uint32_t transfer(uint8_uint32 dataOut);
	void beginTransfer(void); // sets SPISettings and pulls SYNC low
	void endTransfer(void);   // pulls SYNC high to latch data
	
	~AD5761R(void);
	
private:
	uint8_uint32 dataCtrl_;	   // data to write to control register
	const uint8_t csDAC_; // SPI chip-select pin (shared among all DACs)
	SPISettings SPISettings_;  // SPI settings for the DAC
	
	
};

#endif // !AD5761R_h