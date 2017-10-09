#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#include <cstdlib>
#include <stdio.h>   // Standard input/output definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <pthread.h> // This uses POSIX Threads
#include <signal.h>
#include <iostream>
#include "mavlink/v1.0/common/mavlink.h"
#include "crc.hh"
#include <sstream>
#include <time.h>
// The following two non-standard baudrates should have been defined by the system
// If not, just fallback to number
#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif


// Status flags
#define SERIAL_PORT_OPEN   1;
#define SERIAL_PORT_CLOSED 0;
#define SERIAL_PORT_ERROR -1;


/*
 * Serial Port Class
 *
 * This object handles the opening and closing of the offboard computer's
 * serial port over which we'll communicate.  It also has methods to write
 * a byte stream buffer.  MAVlink is not used in this object yet, it's just
 * a serialization interface.  To help with read and write pthreading, it
 * gaurds any port operation with a pthread mutex.
 */

class Serial_Port
{

public:

	int  fd;
	pthread_mutex_t  lock;
	pthread_mutex_t lock_s;
	bool debug;
	const char *uart_name;
	int  baudrate;
	int  status;
	int wrong_packets_counter;

	Serial_Port();
	Serial_Port(const char *uart_name_, int baudrate_);
	~Serial_Port();

	int open_serial();
	int close_serial();

	int _read_port_server(char *data, int size);
	int _write_port_server(char *data, int size);
	
	int  _open_port(const char* port);
	bool _setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);
	int  _read_port(uint8_t &cp);
	int _write_port(char *buf, unsigned len);
	

	int _compute_CRC(char *data); //Returns true if the compute was well. New_message contains the message plus the crc
	bool _check_CRC(char *data);

};



#endif // SERIAL_PORT_H_


