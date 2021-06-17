# AsyncSoftSerial
A class to facilitate a completely asynchronous software serial port

Define a controlling variable with:

    AsyncSoftSerial name;

In setup assign and configure the "Serial Port" thus (as an example):

    name.begin( 2, 4, 38400, serial_buffer, SERIAL_BUFFER );

where (in this example), 2 is the Rx pin, 4 is the Tx pin, 38400 is the baud,
serial_buffer is a byte array to be used as the buffer (which is SERIAL_BUFFER bytes long).
There is an optional parameter to define the Parity requirement at the end.

Then use in (mostly) the same way as Serial, eg:

    name.available()
    name.availableForWrite()
    name.peek()
    name.read()
    name.write( x )

etc.

So the *obvious* question is "How reliable is it?"  To which the honst answer is "it depends".

At lower speeds it's pretty good, as the time per bit gets smaller (baud rate rising) the overlap
created by being able to send and receive *at the same time* can cause the receiving code to
lose its timing accuracy.

In testing on a 16 MHz AVR Atmel 328 sending and receiving at 38400 (using another AVR as a simple
"echo" device) is solid at "human typing speeds", but cutting and pasting whole lines of text is
not 100% reliable.
