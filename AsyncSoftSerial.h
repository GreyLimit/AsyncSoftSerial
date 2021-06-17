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
//	AsyncSoftSerial
//
//	A module to enable a fully asynchronous additional serial
//	line to an AVR MCU driven entirely through interrupts.
//

#ifndef _ASYNCSOFTSERIAL_H_
#define _ASYNCSOFTSERIAL_H_

//
//	Include our Faster pin logic.
//
#include "FasterPinIO.h"

//
//	Define the class used to capture this function to enable
//	it to be used, logcially, in place of the UART based hardware
//	solution.
//

class AsyncSoftSerial : public Stream {
public:
	//
	//	Define an enumerated type to represent the possible parity
	//	options.
	//
	enum SerialParity {
		Parity_None,
		Parity_Odd,
		Parity_Even
	};

private:
	//
	//	Save the key hardware facts about the serial port
	//	(Provided in the contructor routine).
	//
	FasterPinIO	_Rx,		// Receive Pin
			_Tx;		// Transmit Pin
		
	//
	//	Capture the specific operational details of the
	//	connection (provided by the begin() routine).
	//
	//	For the moment, this is restricted to supplying
	//	the baud rate and  buffer address space.  In future
	//	this could include the nature of the serial data
	//	and the posibility for half duplex operation (reducing
	//	the counter requirements from two to one).
	//
	//	But not yet.
	//
	byte		*_buffer,
			//
			//	Total buffer size
			//
			_buf_size,
			//
			//	The Input section of the buffer
			//
			_buf_in_in,
			_buf_in_out,
			_buf_in_len,
			//
			//	The output section of the buffer
			//
			_buf_out_in,
			_buf_out_out,
			_buf_out_len;

	//
	//	Define four routines which add and remove data from
	//	the circular buffer.
	//
	bool buffer_rx_add( byte data );
	bool buffer_rx_get( byte *data );
	bool buffer_tx_add( byte data );
	bool buffer_tx_get( byte *data );

	//
	//	The data items are brought into the data structure
	//	to facilitate the operation of the virtual UART.
	//
	//	The terms "Half Bit" and "Whole Bit" are explained
	//	in the implementation file.
	//
	//	Receive Timing data
	//
	byte		_rx_whole_count,	// 10 to 255
			_rx_whole_scale;	// scale factor

	//
	//	Transmit Timing data
	//
	byte		_tx_whole_count,	// 10 to 255
			_tx_whole_scale;	// scale factor

	//
	//	Parity requirements?
	//
	SerialParity	_parity;

	//
	//	The following fields are used to receive data from the
	//	Rx Pin.
	//
	byte		_RxByte,
			_RxCount;

	//
	//	The following fields are used to transmit a byte of data.
	//
	byte		_TxByte,
			_TxCount;

	//
	//	The following words are used to count errors and issues
	//	detected.
	//
	word		_RxError,		// Stop/Start bit errors.
			_RxDropped,		// Input buffer full, data lost.
			_TxDropped;		// Output buffer full, data lost.
	
public:
	AsyncSoftSerial( void );
	
	void begin( byte Rx, byte Tx, unsigned long baud, byte *buffer, byte len, SerialParity parity = Parity_None );
	void end( void );

	virtual int available( void );
	virtual int peek( void );
	virtual int read( void );
	virtual int availableForWrite( void );
	virtual void flush( void );
	virtual size_t write( uint8_t data );
	
	inline size_t write( unsigned long n )	{ return( write( (uint8_t)n )); }
	inline size_t write( long n )		{ return( write( (uint8_t)n )); }
	inline size_t write( unsigned int n )	{ return( write( (uint8_t)n )); }
	inline size_t write( int n )		{ return( write( (uint8_t)n )); }

	using Print::write; // pull in write(str) and write(buf, size) from Print

	operator bool() { return( true ); }
	
	//
	//	Define routines which will manage the
	//	use of the interrupts with this serial port.
	//
	void enable_receive( void );
	void disable_receive( void );
	void enable_send( void );
	void disable_send( void );

	//
	//	These are the Interrupt Routines used
	//	to handle the simulated UART operation.
	//
	//	Do Not Call directly.
	//
	void edge_isr( void );
	void recv_isr( void );
	void send_isr( void );
};


#endif

//
//	EOF
//
