///
///	AsyncSoftSerial - A software UART implementation
///
///	Copyright (C) 2021  Jeff Penfold, jeff.penfold@googlemail.com
///
///	This program is free software: you can redistribute it and/or modify
///	it under the terms of the GNU General Public License as published by
///	the Free Software Foundation, either version 3 of the License, or
///	(at your option) any later version.
///
///	This program is distributed in the hope that it will be useful,
///	but WITHOUT ANY WARRANTY; without even the implied warranty of
///	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
///	GNU General Public License for more details.
///
///	You should have received a copy of the GNU General Public License
///	along with this program.  If not, see <https://www.gnu.org/licenses/>.
///

//
//	AsyncSoftSerial Implementation
//

#include "Arduino.h"

#include "AsyncSoftSerial.h"

//
//	Debugging
//	=========
//
//	To assist with the receive component of the Software
//	UART the macro SHOW_UART_CLOCK should be defined with
//	the number of a pin which will be used to generate a
//	*representation* of the internal clocking used when
//	reading in a byte of data from the input pin.
//
//	This can be defined here or on the command line.
//
//#define SHOW_UART_CLOCK 8

//
//	Define the pin we will use for showing input
//	event edges etc..
//
#ifdef SHOW_UART_CLOCK
#include "FasterPinIO.h"
static FasterPinIO debug_pin;
#endif

//
//	Generic constants and syntax sugar
//	==================================
//
#ifndef FUNC
#define FUNC(a)		(*(a))
#endif


//
//	Progmem access support
//	======================
//
//	Using simple syntax wrapping
//
#include <avr/pgmspace.h>
#define READ_PROG_BYTE(v)	(byte)pgm_read_byte(&(v))

//
//	Object Parallelism and Interrupts
//	=================================
//
//	It would be possible to implement half as many Software
//	Serial ports as there are available interrupt driving counters,
//	but there are not that many available.
//
//	It *might* be possible to implement a software solution
//	where a single counter could be used to service multiple
//	counter "clients".  This option will be investigated.
//
//	At the moment, and for the purpose of establishing software
//	compliance (i.e. does this actually emulate a functional
//	hardware UART) the software will be restricted to the use
//	of two counters, and each software UART can have its send and
//	receive enabled as required (at the expense of removing the
//	counter from its previous use).
//

//
//	Create symbolic names for the specific counters and associated
//	features that we shall be using in this code.
//
//	Note:	The 8 bit timers are not identical in the
//		specifics of the scale factors.  The following
//		table details possible scale factors, and the
//		appropriate register value for each of the
//		8 bit times (numbers 0 and 2).
//
//		Pre-scaler	Counter		Counter
//		values		0		2
//
//		Disabled	0		0
//		1		1		1
//		8		2		2
//		32				3
//		64		3		4
//		128				5
//		256		4		6
//		1024		5		7
//
//	These are mapped onto three bits of the timer
//	register CSn2:CSn1:CSn0 where 0 is disabled,
//	the the values shown below create the specified
//	clock scale factor.
//

//
//	Map receiver timer symbols onto target counter hardware
//
#define HW_Rx_TIMER		0
#define HW_TCCR_Rx_A		TCCR0A
#define HW_TCCR_Rx_B		TCCR0B
#define HW_TIMER_Rx_COMPA_vect	TIMER0_COMPA_vect
#define HW_TCNT_Rx		TCNT0
#define HW_OCR_Rx_A		OCR0A
#define HW_WGM_Rx_1		WGM01
#define HW_CS_Rx_0		CS00
#define HW_CS_Rx_1		CS01
#define HW_TIMSK_Rx		TIMSK0
#define HW_OCIE_Rx_A		OCIE0A
#define HW_TIRF_Rx		TIFR0
#define HW_TIRF_Rx_CLEAR	0x07
#define HW_Rx_PS_DISABLED	0
#define HW_Rx_PS_1		1
#define HW_Rx_PS_8		2
#define HW_Rx_PS_64		3
#define HW_Rx_PS_256		4
#define HW_Rx_PS_1024		5
#define HW_Rx_PS_PIN_FALL	6
#define HW_Rx_PS_PIN_RISE	7

//
//	Map sender timer symbols onto target counter hardware
//
#define HW_Tx_TIMER		2
#define HW_TCCR_Tx_A		TCCR2A
#define HW_TCCR_Tx_B		TCCR2B
#define HW_TIMER_Tx_COMPA_vect	TIMER2_COMPA_vect
#define HW_TCNT_Tx		TCNT2
#define HW_OCR_Tx_A		OCR2A
#define HW_WGM_Tx_1		WGM21
#define HW_CS_Tx_0		CS20
#define HW_CS_Tx_1		CS21
#define HW_TIMSK_Tx		TIMSK2
#define HW_OCIE_Tx_A		OCIE2A
#define HW_TIRF_Tx		TIFR2
#define HW_TIRF_Tx_CLEAR	0x07
#define HW_Tx_PS_DISABLED	0
#define HW_Tx_PS_1		1
#define HW_Tx_PS_8		2
#define HW_Tx_PS_32		3
#define HW_Tx_PS_64		4
#define HW_Tx_PS_128		5
#define HW_Tx_PS_256		6
#define HW_Tx_PS_1024		7


//
//	As we are using only one counter each for receive and
//	sending management we need to keep track of which software
//	UARTs are being serviced.  The following variables do this.
//
static AsyncSoftSerial	*active_send = NULL,
			*active_recv = NULL;

//
//	The following flag is used to indicate if the sending
//	ISR is currently active.
//
volatile bool		busy_sending = false;

//
//	The following ISRs provide the immediate code executed by
//	the interrupt system, and simply direct the execution to
//	the software UART being serviced for that action.
//
static void edge_isr( void ) { if( active_recv ) active_recv->edge_isr(); }

ISR( HW_TIMER_Rx_COMPA_vect ) { if( active_recv) active_recv->recv_isr(); }

ISR( HW_TIMER_Tx_COMPA_vect ) { if( active_send ) active_send->send_isr(); }


//
//	UART Emulation, Timing is everything.
//	=====================================
//
//	Thinking about how baud rates are matched into the clock speed
//	of the CPU.
//

//
//				Clock	20	16	8	4	1
//				(MHz)
//			Time
//	Baud Rate	per bit	Clocks
//	(bits/sec)	(us)	(per bit)
//
//	4800		208.3		4167	3333	1667	833	208				
//	9600		104.2		2083	1667	833	417	104
//	14400		69.4		1389	1111	556	278	69
//	19200		52.1		1042	833	417	208	52
//	38400		26.0		521	417	208	104	26
//	56000		17.9		357	286	143	71	18
//	115200		8.7		174	139	69	35	9
//
//	Clock ticks per bits are rounded to nearest whole tick.
//

//
//	Thought / Concept
//
//	Perform an initial wait of "half tick" to place the following "whole ticks"
//	"in the middle" of each bit of data arriving.  Thus:
//
//		 Time
//		(*tpb)	Count	Event
//		------	-----	-----
//		0	n/a	Leading Edge Transmission
//		0.5	0	Start Bit = 0
//		1.5	1	Bit 0
//		2.5	2	Bit 1
//		3.5	3	Bit 2
//		4.5	4	Bit 3
//		5.5	5	Bit 4
//		6.5	6	Bit 5
//		7.5	7	Bit 6
//		8.5	8	Bit 7
//		9.5	9	Stop Bit = 1
//
//	Enable ISR_EDGE as "Falling edge triggered" ISR on the Input Pin
//
//	ISR_EDGE is called, we know a byte is arriving:
//		Enable ISR_RECV with timed countdown period of a "half tick"
//		Disable ISR_EDGE
//		Set ISR_RECV counter to 0
//		Set RxByte to 0
//		Done
//
//	ISR_RECV is called
//		read input pin -- done here to maximise time consistency.
//		switch on counter
//			case counter is 0
//				Check start bit is 0
//				Enable ISR_RECV with timed countdown period of "0.5*tpb" us
//			case counter is 9
//				Check stop bit is 1
//				save RxByte in input buffer
//				Disable ISR_RECV
//				Enable ISR_EDGE as "Falling edge triggered" ISR on the Input Pin
//			other counter values 1 to 8
//				add input pin value as MSB in RxByte
//		end switch
//		add 1 to counter
//		Done.
//

//
//	The following data structure (set up specifically by
//	the target MCU clock frequency) dictates how the timing
//	interrupt needs to be configured to allow the system to
//	accurately "clock in" data sent to the Rx pin.
//
//	This table (arbitarily) does not code for any speed where
//	the bit rate allows for less than 100 instructions between
//	bits.  This does not mean that these cannot be added, just
//	this I feel that there is inadequate time between events
//	for the system to be reliable.
//
//	We are aiming to use an 8 bit timer with scaling where necessary
//	(at the moment).
//
//	The "Clocking in" of data is divided into two steps
//
typedef struct _clock_timing {
	//
	//	The BAUD Rate the clock data represents.
	//	(value stored is rounded value of baud rate
	//	divided by 1600 Hz)
	//
#define BAUD_SCALE	1600
	//
	//	Thus:	Baud	Value
	//		----	-----
	//		4800	3
	//		9600	6
	//		14400	9
	//		19200	12
	//		38400	24
	//		56000	35
	//		115200	72
	//
#define BAUD_4800	3
#define BAUD_9600	6
#define BAUD_14400	9
#define BAUD_19200	12
#define BAUD_38400	24
#define BAUD_56000	35
#define BAUD_115200	72
	//
	byte	baud;
	//
	//	Timing specifics for the receiving code.
	//
	byte	rx_whole_count,		// 10 to 255
		rx_whole_scale;		// scale factor(*)
	//
	//	Timing specifics for the transmission code.
	//
	byte	tx_whole_count,		// 10 to 255
		tx_whole_scale;		// scale factor(*)
	
	//
	//	(*)	Scale factors are stored as the appropiate
	//		bit map which needs to be applied when
	//		setting up the interrupt timer.
	
	//
	//	End of structure
	//
} clock_timing;


#if F_CPU == 1000000
//
//	1 MHz Clock Frequency
//	---------------------
//
//	From the above baud rate table, the following extract shows
//	the specific counter pre-scaler and count required for both
//	the whole and half bit counts.
//
//	The Half Bit count data will have been adjusted to minimise
//	(average out) any compounded errors present in the Whole Bit
//	count data.
//
//	Baud 	T/bit	Clocks		Rx	Tx
//	Rate	(us)	(per bit)	Whole	Whole
//
//	4800	208.3	208		208/1	208/1
//
static const clock_timing baud_rate[] PROGMEM = {
	{	BAUD_4800,	208,HW_Rx_PS_1,	208,HW_Tx_PS_1	},
//
	{	0,		0, 0,		0, 0		}
};

#elif F_CPU == 4000000
//
//	4 MHz Clock Frequency
//	---------------------
//
//	Baud 	T/bit	Clocks		Rx	Tx
//	Rate	(us)	(per bit)	Whole	Whole
//
//	4800	208.3	833		104/8	104/8
//	9600	104.2	417		52/8	52/8
//	14400	69.4	278		35/8	35/8
//	19200	52.1	208		208/1	208/1
//
static const clock_timing baud_rate[] PROGMEM = {
	{	BAUD_4800,	104,HW_Rx_PS_8,	104,HW_Tx_PS_8	},
	{	BAUD_9600,	52,HW_Rx_PS_8,	52,HW_Tx_PS_8	},
	{	BAUD_14400,	35,HW_Rx_PS_8,	35,HW_Tx_PS_8	},
	{	BAUD_19200,	208,HW_Rx_PS_1,	208,HW_Tx_PS_1	},
//
	{	0,		0, 0,		0, 0		}
};

#elif F_CPU == 8000000
//
//	8 MHz Clock Frequency
//	---------------------
//
//	Baud 	T/bit	Clocks		Rx	Tx
//	Rate	(us)	(per bit)	Whole	Whole
//
//	4800	208.3	1667		208/8	208/8
//	9600	104.2	833		104/8	104/8
//	14400	69.4	556		70/8	70/8
//	19200	52.1	417		52/8	52/8
//	38400	26.0	208		208/1	208/1
//
static const clock_timing baud_rate[] PROGMEM = {
	{	BAUD_4800,	208,HW_Rx_PS_8,	208,HW_Tx_PS_8	},
	{	BAUD_9600,	104,HW_Rx_PS_8,	104,HW_Tx_PS_8	},
	{	BAUD_14400,	70,HW_Rx_PS_8,	70,HW_Tx_PS_8	},
	{	BAUD_19200,	52,HW_Rx_PS_8,	52,HW_Tx_PS_8	},
	{	BAUD_38400,	208,HW_Rx_PS_1,	208,HW_Tx_PS_1	},
//
	{	0,		0, 0,		0, 0		}
};
//
#elif F_CPU == 16000000
//
//	16 MHz Clock Frequency
//	----------------------
//
//	Baud 	T/bit	Clocks		Rx	Tx
//	Rate	(us)	(per bit)	Whole	Whole
//
//	4800	208.3	3333		52/64	104/32
//	9600	104.2	1667		208/8	208/8
//	14400	69.4	1111		139/8	139/8
//	19200	52.1	833		104/8	104/8
//	38400	26.0	417		52/8	52/8
//	56000	17.9	286		36/8	36/8
//
static const clock_timing baud_rate[] PROGMEM = {
	{	BAUD_4800,	52,HW_Rx_PS_64,	104,HW_Tx_PS_32	},
	{	BAUD_9600,	208,HW_Rx_PS_8,	208,HW_Tx_PS_8	},
	{	BAUD_14400,	139,HW_Rx_PS_8,	139,HW_Tx_PS_8	},
	{	BAUD_19200,	104,HW_Rx_PS_8,	104,HW_Tx_PS_8	},
	{	BAUD_38400,	52,HW_Rx_PS_8,	52,HW_Tx_PS_8	},
	{	BAUD_56000,	36,HW_Rx_PS_8,	36,HW_Tx_PS_8	},
//
	{	0,		0, 0,		0, 0		}
};
//
#elif F_CPU == 20000000
//
//	20 MHz Clock Frequency
//	----------------------
//
//	Baud 	T/bit	Clocks		Rx	Tx
//	Rate	(us)	(per bit)	Whole	Whole
//
//	4800	208.3	4167		65/64	130/32
//	9600	104.2	2083		33/64	65/32
//	14400	69.4	1389		174/8	174/8
//	19200	52.1	1042		130/8	130/8
//	38400	26.0	521		65/8	65/8
//	56000	17.9	357		45/8	45/8
//
static const clock_timing baud_rate[] PROGMEM = {
	{	BAUD_4800,	65,HW_Tx_PS_64,130,HW_Tx_PS_32	},
	{	BAUD_9600,	33,HW_Tx_PS_64,	65,HW_Tx_PS_32	},
	{	BAUD_14400,	174,HW_Tx_PS_8,	174,HW_Tx_PS_8	},
	{	BAUD_19200,	130,HW_Tx_PS_8,	130,HW_Tx_PS_8	},
	{	BAUD_38400,	65,HW_Tx_PS_8,	65,HW_Tx_PS_8	},
	{	BAUD_56000,	45,HW_Tx_PS_8,	45,HW_Tx_PS_8	},
//
	{	0,		0, 0,		0, 0		}
};
//
#else

//
//	The target MCU clock frequency has not been accounted for.
//
#error "MCU Clock speed calculation needs to be calculate for this clock rate."

#endif



//
//	Static, non-Object based, Support Routines
//	==========================================
//

//
//	Baud rate lookup routines
//
static const clock_timing *find_baud( unsigned long baud ) {
	byte	byte_baud;

	byte_baud = baud / BAUD_SCALE;
	for( const clock_timing *look = baud_rate; READ_PROG_BYTE( look->baud ); look++ ) {
		if( READ_PROG_BYTE( look->baud ) == byte_baud ) return( look );
	}
	return( NULL );
}
static const clock_timing *find_default_baud( void ) {
	return( baud_rate );
}

//
//	Set up the Interrupt Service Routines
//	=====================================
//

//
//	Define routines used to enable and disable the Edge Detecting
//	ISR
//
static inline void enable_edge_isr( byte pin ) {
	//
	//	Clear the "Pin Change" interrupt flag before we
	//	attach an interrupt routine to the event.
	//
	EIFR = 1 << digitalPinToInterrupt( pin );
	attachInterrupt( digitalPinToInterrupt( pin ), edge_isr, FALLING );
}
static inline void disable_edge_isr( byte pin ) {
	//
	//	Clear the "Pin Change" interrupt flag before we
	//	detach the interrupt routine from the event.
	//
	EIFR = 1 << digitalPinToInterrupt( pin );
	detachInterrupt( digitalPinToInterrupt( pin ));
}

static inline void enable_recv_isr( byte count, byte prescaler ) {
	//
	//	Clear Counter configuration
	//
	HW_TCNT_Rx	= 0;
	HW_OCR_Rx_A	= 0;
	HW_TCCR_Rx_A	= 0;
	HW_TCCR_Rx_B	= 0;
	HW_TIMSK_Rx	= 0;
	//
	//	Clear Pending Interrupts
	//
	HW_TIRF_Rx	= HW_TIRF_Rx_CLEAR;
	//
	//	Set up new configuration
	//
	HW_TCNT_Rx	= 0;
	HW_OCR_Rx_A	= count;
	HW_TCCR_Rx_A	= bit( HW_WGM_Rx_1 );
	HW_TCCR_Rx_B	= prescaler;
	HW_TIMSK_Rx	= bit( HW_OCIE_Rx_A );
}
static inline void disable_recv_isr( void ) {
	//
	//	Clear Counter configuration
	//
	HW_TCNT_Rx	= 0;
	HW_OCR_Rx_A	= 0;
	HW_TCCR_Rx_A	= 0;
	HW_TCCR_Rx_B	= 0;
	HW_TIMSK_Rx	= 0;
	//
	//	Clear Pending Interrupts
	//
	HW_TIRF_Rx	= HW_TIRF_Rx_CLEAR;
}
static inline void enable_send_isr( byte count, byte prescaler ) {
	//
	//	Clear Counter configuration
	//
	HW_TCNT_Tx	= 0;
	HW_OCR_Tx_A	= 0;
	HW_TCCR_Tx_A	= 0;
	HW_TCCR_Tx_B	= 0;
	HW_TIMSK_Tx	= 0;
	busy_sending	= false;
	//
	//	Clear Pending Interrupts
	//
	HW_TIRF_Tx	= HW_TIRF_Tx_CLEAR;
	//
	//	Set up new configuration
	//
	HW_TCNT_Tx	= 0;
	HW_OCR_Tx_A	= count;
	HW_TCCR_Tx_A	= bit( HW_WGM_Tx_1 );
	HW_TCCR_Tx_B	= prescaler;
	HW_TIMSK_Tx	= bit( HW_OCIE_Tx_A );
	busy_sending	= true;
}
static inline void disable_send_isr( void ) {
	//
	//	Clear Counter configuration
	//
	HW_TCNT_Tx	= 0;
	HW_OCR_Tx_A	= 0;
	HW_TCCR_Tx_A	= 0;
	HW_TCCR_Tx_B	= 0;
	HW_TIMSK_Tx	= 0;
	busy_sending	= false;
	//
	//	Clear Pending Interrupts
	//
	HW_TIRF_Tx	= HW_TIRF_Tx_CLEAR;
}

//
//	Parity creation and checking code
//	=================================
//
//	An array of 128 bits representing the values
//	0 to 127.  Where a bit is set then that value
//	currently has even parity, where there is no
//	bit set then that value has odd parity.
//
static const byte parity_data[ 16 ] PROGMEM = {
	105, 150, 150, 105,
	150, 105, 105, 150,
	150, 105, 105, 150,
	105, 150, 150, 105
};

//
//	Function returns true when a 7 bit value needs a parity
//	bit to make an ODD parity 8 bit value.
//
static inline bool parity_bit( byte value ) {
	return(( READ_PROG_BYTE( parity_data[( value >> 3 ) & 0x0f ]) >> ( value & 0x07 )) & 1 );
}

//
//	Buffer access and management routines
//	=====================================
//

//
//	bool buffer_rx_add( byte data )
//	-------------------------------
//
//	Add byte "data" to the input part of the buffer
//	and return true on success, false otherwise.
//	
bool AsyncSoftSerial::buffer_rx_add( byte data ) {
	byte	i;

	//
	//	Is there space for the byte received?
	//
	if( _buf_in_in == _buf_out_out ) {
		//
		//	The input buffer has caught up with the
		//	output buffer.
		//
		//	If the output buffer is not empty, then this can
		//	not be silently moved along.
		//
		if( _buf_out_len ) return( false );
		//
		//	Push empty output buffer some distance.  Lets
		//	grab half of the available space, but need to keep
		//	in mind the wrapping round effects.
		//
		if( _buf_out_in >= _buf_in_out ) {
			//
			//	The wrap point is between the output and
			//	input parts of the buffer
			//
			i = _buf_size - ( _buf_out_in - _buf_in_out );
		}
		else {
			//
			//	The easy case
			//
			i = _buf_in_out - _buf_out_in;
		}
		//
		//	Now halve and round up.
		//
		i = ( i + 1 ) >> 1;
		//
		//	At this point i the value we are looking for.
		//
		if(( _buf_out_in += i ) >= _buf_size ) _buf_out_in -= _buf_size;
		//
		//	Now make sure the output buffer is still empty.
		//
		_buf_out_out = _buf_out_in;
	}
	//
	//	Is the parity correct for the byte received?
	//
	switch( _parity ) {
		case Parity_Even: {
			if( parity_bit( data ) == (( data & 0x80 ) != 0 )) return( false );
			data &= 0x7f;
			break;
		}
		case Parity_Odd: {
			if( parity_bit( data ) != (( data & 0x80 ) != 0 )) return( false );
			data &= 0x7f;
			break;
		}
		default: break;
	}
	//
	//	save that data byte!
	//
	_buffer[ _buf_in_in++ ] = data;
	if( _buf_in_in >= _buf_size ) _buf_in_in = 0;
	_buf_in_len++;
	//
	//	Done successfully.
	//
	return( true );
}

bool AsyncSoftSerial::buffer_rx_get( byte *data ) {
	if( _buf_in_len ) {
		*data = _buffer[ _buf_in_out++ ];
		if( _buf_in_out >= _buf_size ) _buf_in_out = 0;
		_buf_in_len--;
		return( true );
	}
	return( false );
}
	
//
//	bool buffer_tx_add( byte data )
//	-------------------------------
//
//	Add byte "data" to the output part of the buffer
//	and return true on success, false otherwise.
//	
bool AsyncSoftSerial::buffer_tx_add( byte data ) {
	byte	i;

	if( _buf_out_in == _buf_in_out ) {
		//
		//	The output buffer has caught up with the
		//	input buffer.
		//
		//	If the input buffer is not empty, then this
		//	can not be silently moved along.
		//
		if( _buf_in_len ) return( false );
		//
		//	Push empty input buffer some distance.  Lets
		//	grab half of the available space, but need to keep
		//	in mind the wrapping round effects.
		//
		if( _buf_in_in >= _buf_out_out ) {
			//
			//	The wrap point is between the output and
			//	input parts of the buffer
			//
			i = _buf_size - ( _buf_in_in - _buf_out_out );
		}
		else {
			//
			//	The easy case
			//
			i = _buf_out_out - _buf_in_in;
		}
		//
		//	Now halve and round up.
		//
		i = ( i + 1 ) >> 1;
		//
		//	At this point i the value we are looking for.
		//
		if(( _buf_in_in += i ) >= _buf_size ) _buf_in_in -= _buf_size;
		//
		//	Now make sure the input buffer is still empty.
		//
		_buf_in_out = _buf_in_in;
	}
	//
	//	save that data byte!
	//
	_buffer[ _buf_out_in++ ] = data;
	if( _buf_out_in >= _buf_size ) _buf_out_in = 0;
	_buf_out_len++;
	//
	//	Done successfully.
	//
	return( true );
}

bool AsyncSoftSerial::buffer_tx_get( byte *data ) {
	if( _buf_out_len ) {
		*data = _buffer[ _buf_out_out++ ];
		if( _buf_out_out >= _buf_size ) _buf_out_out = 0;
		_buf_out_len--;
		return( true );
	}
	return( false );
}
	

AsyncSoftSerial::AsyncSoftSerial( void ) {
	//
	//	It is an unassigned serial port.
	//
	//
	//	We do nothing else here as the begin() routine
	//	does the major part of the setup.
	//
}
	
void AsyncSoftSerial::begin( byte Rx, byte Tx, unsigned long baud, byte *buffer, byte len, SerialParity parity ) {
	const clock_timing	*defn;
	
	//
	//	Enable debugging, if requested.
	//
#ifdef SHOW_UART_CLOCK
	debug_pin.setPin( SHOW_UART_CLOCK );
	debug_pin.output();
	debug_pin.low();
#endif

	//
	//	Initialise the IO pins.
	//
	_Rx.setPin( Rx );
	_Tx.setPin( Tx );
	_Rx.input();
	_Tx.output();
	_Tx.high();

	//
	//	Set up the baud rate.
	//
	if(( defn = find_baud( baud )) == NULL ) {
		//
		//	No such baud, we will pick a default setting
		//	(probably 9600, which seems like a good choice).
		//
		defn = find_default_baud();
	}
	//
	//	Now fill out the essential variables required to get this
	//	virtual device off the ground...
	//
	_rx_whole_count = READ_PROG_BYTE( defn->rx_whole_count );
	_rx_whole_scale = READ_PROG_BYTE( defn->rx_whole_scale );
	_tx_whole_count = READ_PROG_BYTE( defn->tx_whole_count );
	_tx_whole_scale = READ_PROG_BYTE( defn->tx_whole_scale );
	_buffer = buffer;
	_buf_size = len;
	//
	//	The Input section of the buffer
	//
	_buf_in_in = _buf_in_out = 0;
	_buf_in_len = 0;
	//
	//	The output section of the buffer
	//
	_buf_out_in = _buf_out_out = _buf_size >> 1;
	_buf_out_len = 0;
	//
	//	Set parity requirements.
	//
	_parity = parity;
	//
	//	Reset the send/receive bytes and counters
	//
	_RxByte = 0;
	_RxCount = 0;
	_TxByte = 0;
	_TxCount = 0;
	//
	//	Reset the error counters.
	//
	_RxError = 0;
	_RxDropped = 0;
	_TxDropped = 0;
	//
	//	Lets kick off send and receive as if we were the
	//	only software serial port defined.
	//
	enable_receive();
	enable_send();
	//
	//	Done.
	//
}

void AsyncSoftSerial::end( void ) {
	//
	//	Actually not going to do anything in here.
	//
}

int AsyncSoftSerial::available( void ) {
	return( _buf_in_len );
}

int AsyncSoftSerial::peek( void ) {
	if( _buf_in_len ) return( _buffer[ _buf_in_out ]);
	return( -1 );
}

int AsyncSoftSerial::read( void ) {
	byte	v;
	
	noInterrupts();
	if( buffer_rx_get( &v )) {
		interrupts();
		return( v );
	}
	interrupts();
	return( -1 );
}

int AsyncSoftSerial::availableForWrite( void ) {
	//
	//	We choose to lie about available space and report
	//	only half the available space.
	//
	return(( _buf_size - ( _buf_in_len + _buf_out_len )) >> 1 );
}

void AsyncSoftSerial::flush( void ) {
	//
	//	All we need to do is wait until busy_sending is false.
	//
	while( busy_sending );
}

size_t AsyncSoftSerial::write( uint8_t data ) {
	size_t	r;

	//
	//	Do the parity thing.
	//
	switch( _parity ) {
		case Parity_Even: {
			if( !parity_bit( data )) data = 0x80|( data & 0x7f );
			break;
		}
		case Parity_Odd: {
			if( parity_bit( data )) data = 0x80|( data & 0x7f );
			break;
		}
		default: break;
	}
	//
	//	Append data to the tail of the output
	//	buffer, and note if we succeeded (or not).
	//
	noInterrupts();
	if( buffer_tx_add( data )) {
		r = sizeof( data );
		//
		//	We know that we have added the data to the
		//	buffer.  Do we need to kick off data sending?
		//
		if(( active_send == this ) && !busy_sending ) {
			//
			//	The send ISR is ours, there is data to send
			//	but the routine is inactive.  Get the next byte
			//	and Kick it off.
			//
			(void)buffer_tx_get( &_TxByte );
			_TxCount = 11;
			enable_send_isr( _tx_whole_count, _tx_whole_scale );
		}
	}
	else {
		_TxDropped++;
		r = 0;
	}
	interrupts();
	return( r );
}

//
//	Define routines which will manage the
//	use of the interrupts with this serial port.
//
void AsyncSoftSerial::enable_receive( void ) {
	//
	//	If we do not already have the ISR pointing to
	//	us...
	//
	if( active_recv != this ) {
		//
		//	Do we need to disable another serial port?
		//
		if( active_recv != NULL ) active_recv->disable_receive();
		//
		//	Link ourselves in
		//
		noInterrupts();
		active_recv = this;
		enable_edge_isr( _Rx.pin());
		interrupts();
	}	
}
void AsyncSoftSerial::disable_receive( void ) {
	//
	//	If we have the ISR then release it.
	//
	if( active_recv == this ) {
		noInterrupts();
		disable_edge_isr( _Rx.pin());
		disable_recv_isr();
		active_recv = NULL;
		interrupts();
	}
}

void AsyncSoftSerial::enable_send( void ) {
	//
	//	If we do not already have the ISR pointing to
	//	us...
	//
	if( active_send != this ) {
		//
		//	Do we need to disable another serial port?
		//
		if( active_send != NULL ) active_send->disable_send();
		//
		//	Link ourselves in
		//
		noInterrupts();
		active_send = this;
		//
		//	Prod the output queue into action, set
		//	pending activity
		//
		if( _buf_out_len ) {
			//
			//	There is data to send, so send it.
			//
			(void)buffer_tx_get( &_TxByte );
			_TxCount = 11;
			enable_send_isr( _tx_whole_count, _tx_whole_scale );
		}
		else {
			//
			//	Wait for something to send.
			//
			_TxByte = 0;
			_TxCount = 0;
		}
		interrupts();
	}	
}
void AsyncSoftSerial::disable_send( void ) {
	//
	//	If we have the ISR then release it
	//
	if( active_send == this ) {
		noInterrupts();
		disable_send_isr();
		active_send = NULL;
		interrupts();
	}
}

//
//	Interrupt routines that implement the actions
//	of the UARTS.
//
void AsyncSoftSerial::edge_isr( void ) {

	//
	//	This is the "Version 2" edge detect ISR.
	//
	//	The thought process is that rather than trying to put the sample
	//	point "in the middle" of each data bit, it is better to try and
	//	place it as close as possible to the start of the data bit.  Then
	//	any execution delay in the ISR slides this point towards the tail
	//	the data bit, and (hopefully) not out of it!  So:
	//
	//	Code optimised for minimum execution time.
	//

#ifdef SHOW_UART_CLOCK
	//
	//	Edge detect - send clocking debug pin high.
	//
	debug_pin.high();
#endif

	enable_recv_isr( _rx_whole_count, _rx_whole_scale );
	disable_edge_isr( _Rx.pin());
	_RxCount = 0;
	_RxByte = 0;
	
}

void AsyncSoftSerial::recv_isr( void ) {
	bool	input;

#ifdef SHOW_UART_CLOCK
	//
	//	Toggle and show clock edges.
	//
	debug_pin.write( _RxCount & 1 );
#endif

	//
	//	Read input pin.
	//
	input = _Rx.read();
	switch( _RxCount ) {
		case 8: {
			//
			//	Bit count 8 is the stop bit, this should be 1, if it is
			//	the add the byte to the Rx queue.
			//
			//	If not, drop it and increment the error count.
			//
			if( input ) {
				//
				//	Add to the input buffer
				//
				if( !buffer_rx_add( _RxByte )) _RxDropped++;
			}
			else {
				 _RxError++;
			}
			_RxCount++;
			break;
		}
		case 9: {
			//
			//	Bit count 9 should be the tail of the stop bits.
			//
			//	Still in the stop bit?
			//
			if( !input ) _RxError++;
			//
			//	Now setup ready for the next byte to arrive.
			//
			disable_recv_isr();
			enable_edge_isr( _Rx.pin());
			
#ifdef SHOW_UART_CLOCK
				//
				//	Toggle and show clock edges.
				//
				debug_pin.low();
#endif

			break;
		}
		default: {
			//
			//	Counts 0 to 7 (inclusive) are the data bits which
			//	we collect.
			//
			_RxByte >>= 1;
			if( input ) _RxByte |= 0x80;
			_RxCount++;
			break;
		}
	}
}

void AsyncSoftSerial::send_isr( void ) {
	//
	//	The send ISR is simpler, the Tx Counter starts at 11, and
	//	and finishes at zero (when it loads the next character to
	//	output (if there is one).
	//
	//	Step 11 is the start bit (output goes LOW)
	//
	//	Steps 10..3 are the data bits, LSB first, 1 -> HIGH, 0 -> LOW.
	//
	//	Steps 2..1 are the stop bits (output goes HIGH).
	//
	//	Step 0 end of this byte.
	//
	switch( _TxCount ) {
		case 11: {
			//
			//	11: Output the "Start bit" taking the output pin low.
			//
			_Tx.low();
			_TxCount--;
			break;
		}
		case 2:
		case 1: {
			//
			//	2, 1: Stop bits.  Take the output high.
			//
			_Tx.high();
			_TxCount--;
			break;
		}
		case 0: {
			//
			//	Count 0, Re-load for next bit, if there is one.
			//
			if( buffer_tx_get( &_TxByte )) {
				_TxCount = 11;
			}
			else {
				//
				//	Buffer is empty, kill off the send interrupt.
				//
				disable_send_isr();
			}
			break;
		}
		default: {
			//
			//	For bit counts 10 to 3 we stream the bits out of
			//	transmit buffer.
			//
			_Tx.write( _TxByte & 1 );
			_TxByte >>= 1;
			_TxCount--;
			break;
		}
	}
}

//
//	EOF
//

