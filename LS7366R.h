/*=========================================================================//

	LS7366R Class
	
	This class is used to configure and use the LS7366R Quadrature Coutner
	
	Designed for use with Teensy 3.X and Motor Control Board (Rev 1.X)
	
	
	Trevor Bruns
	
	Changelog-
		2/11/16: Initial creation
		3/14/17: Changed names to conform to convention
				 Removed unused code
        4/18/17: Updated function names and added isConfigured()

//=========================================================================*/

#ifndef LS7366R_h
#define LS7366R_h

#include <SPI.h>

// OP Codes
#define CLR_MDR0 0x08
#define CLR_MDR1 0x10
#define CLR_CNTR 0x20
#define CLR_STR 0x30
#define READ_MDR0 0x48
#define READ_MDR1 0x50
#define READ_CNTR 0x60
#define READ_OTR 0x68
#define READ_STR 0x70
#define WRITE_MDR1 0x90
#define WRITE_MDR0 0x88
#define WRITE_DTR 0x98
#define LOAD_CNTR 0xE0
#define LOAD_OTR 0xE4

// MDR0 Options
#define QUAD_0X	0x00 // non-quadrature count mode (A = clock, B = direction)
#define QUAD_1X 0x01 // one count per quadrature cycle
#define QUAD_2X 0x02 // two counts per quadrature cycle
#define QUAD_4X	0x03 // four counts per quadrature cycle
#define COUNT_FREE	 0x00  // free-running count mode
#define COUNT_SINGLE 0x04  // single-cycle count mode
#define COUNT_LIMIT	 0x08  // range-limit count mode (limitd between DTR and zero)
#define COUNT_MOD	 0x0C  // modulo-n count mode (count clock divided by n+1, where n = DTR)
#define INDEX_NO		 0x00  // disable index
#define INDEX_LOAD_CNTR	 0x10  // configure index as the "load CNTR" input (transfers DTR to CNTR)
#define INDEX_RST_CNTR	 0x20  // configure index as the "reset CNTR" input (clears CNTR to zero)
#define INDEX_LOAD_OTR	 0x30  // configure index as the "load OTR" input (transfers CNTR to OTR)
#define INDEX_ASYNC  0x00  // Asynchronous index
#define INDEX_SYNC	 0x80  // Synchronous index (overridden in non-quadrature mode)
#define	FILTER_1	 0x00  // Filter clock division factor = 1
#define FILTER_2	 0x80  // Filter clock division factor = 2

// MDR1 Options
#define SIZE_4	0x00  // 4-byte counter mode
#define SIZE_3	0x01  // 3-byte counter mode
#define SIZE_2	0x02  // 2-byte counter mode
#define SIZE_1	0x03  // 1-byte counter mode
#define CNT_ENABLE	0x00  // Enable counting
#define CNT_DISABLE 0x04  // Disable counting
#define NO_FLAGS 0x00  // all flags disabled
#define IDX_FLAG 0x10; // IDX flag
#define CMP_FLAG 0x20; // CMP flag
#define BW_FLAG  0x40; // BW flag
#define CY_FLAG  0x80; // CY flag

#ifndef AD5761R_h
union uint8_uint32
{
	uint8_t byte[4];
	uint32_t value;
};

union uint8_int32
{
    uint8_t byte[4];
    int32_t value;
};
#endif

class LS7366R
{
public:
	LS7366R(uint8_t csPin_);
	~LS7366R(void);

	bool init(void); // configures encoder and enables counting; returns configured_
	int32_t getCount(void);   // returns current count
    bool resetCount(void);    // resets the count to zero
    uint8_t readStatus(void); // returns STR and sets configured_
	bool isConfigured(void);  // return configured_

	// use these only for debugging
	uint8_t read(uint8_t opcode, uint8_t data_out);
	void write(uint8_t opcode, uint8_t data_out);
	void write(uint8_t opcode);

private:
	uint8_t csPin_; // SPI chip-select pin
	SPISettings SPISettings_;	 // SPI settings for the DAC
    bool configured_;
};

#endif // !LS7366R_h
