
#include "Serial_Port.h"

using namespace std;

Serial_Port::Serial_Port(){

	uart_name = (char*) "/dev/ttyUSB0";
	baudrate = 57600;

	// Start mutex
	pthread_mutex_init(&lock, NULL);
	pthread_mutex_init(&lock_s, NULL);

	// Initialize attributes
	debug  = false;
	fd     = -1;
	status = SERIAL_PORT_CLOSED;
}

Serial_Port::Serial_Port(const char *uart_name_ , int baudrate_)
{
	
	uart_name = uart_name_;
	baudrate  = baudrate_;

	// Start mutex
	pthread_mutex_init(&lock, NULL);
	pthread_mutex_init(&lock_s, NULL);

	// Initialize attributes
	debug  = false;
	fd     = -1;
	status = SERIAL_PORT_CLOSED;
}

Serial_Port::~Serial_Port()
{
	// destroy mutex
	pthread_mutex_destroy(&lock);
	pthread_mutex_destroy(&lock_s);
}

//Open the serial port Uart_Name
int Serial_Port::open_serial()
{

	fd = _open_port(uart_name);

	// Check success
	if (fd == -1)
	{
		cout << "[SerialPort] - Failure, could not open port "<<uart_name<<endl;
		return -1;
	}

	bool success = _setup_port(baudrate, 8, 1, false, false);

	if (!success)
	{	
		cout << "[SerialPort] - Failure, could not configure port."<<endl;
		return -1;
	}
	if (fd <= 0)
	{
		cout << "[SerialPort] - Connection attempt to port " << uart_name << " with  "<<baudrate<<" baud, 8N1 failed, exiting."<<endl;
		return -1;
	}

	//cout << "[SerialPort] - Connected to "<<uart_name<<" with "<<baudrate<<" baud,8 data bits, no parity, 1 stop bit (8N1)."<<endl;

	//lastStatus.packet_rx_drop_count = 0;

	status = true;

	return 0;

}


//Close serial port Uart_name
int Serial_Port::close_serial()
{
	status = false;
	if (close(fd))
		return -1;
	
	return 0;
}

// Where the actual port opening happens, returns file descriptor 'fd'
int Serial_Port::_open_port(const char* port)
{
	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

	// Check for Errors
	if (fd == -1)
		return -1;
	else
		fcntl(fd, F_SETFL, 0);
	

	return fd;
}

// Sets configuration, flags, and baud rate
bool Serial_Port::_setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
	// Check file descriptor
	if(!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		return false;
	}

	// Read file descritor configuration
	struct termios  config;
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}

	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//config.c_iflag = IGNPAR | ICRNL;

	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
	//config.c_oflag = 0;

	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif

	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	//config.c_lflag = ICANON;

	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
//	config.c_cflag = 	CRTSCTS | CS8 | CLOCAL | CREAD;

	// One input byte is enough to return from read()
	// Inter-character timer off
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	////struct termios options;
	////tcgetattr(fd, &options);

	// Apply baudrate
	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;

			break;
	}

	 //tcflush(fd, TCIFLUSH);
	// Finally, apply the configuration
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}

	// Done!
	return true;
}

int Serial_Port::_read_port(uint8_t &cp)
{
	// Lock
	pthread_mutex_lock(&lock);

	int result = read(fd, &cp, 1);

	// Unlock
	pthread_mutex_unlock(&lock);

	return result;
}


int Serial_Port::_write_port(char *buf, unsigned len)
{

	// Lock
	pthread_mutex_lock(&lock);

	// Write packet via serial link
	const int bytesWritten = static_cast<int>(write(fd, buf, len));

	// Wait until all data has been written
	tcdrain(fd);

	// Unlock
	pthread_mutex_unlock(&lock);

	return bytesWritten;
}

int Serial_Port::_compute_CRC(char *data)
{

  char *test = data;
	//crcInit();
	crc crc = crcSlow(test, strlen(test));
  char crc32[32] = "";
	char dat[255] = "";
  sprintf(crc32, "||%X|", crc); // concat char
	strcat(dat, crc32);
	strcat (dat, test);
	memcpy(data,dat,strlen(dat));

	return 0;
}

bool Serial_Port::_check_CRC(char *data) 
{
	stringstream ss;
	ss << data;
	std::string raw_msg_received = ss.str();
	//cout << "msg_received: "<<raw_msg_received<<endl;
	raw_msg_received = raw_msg_received.substr(raw_msg_received.find("CMD"),raw_msg_received.length());
	//cout << "msg_received without CRC: "<<raw_msg_received<<endl;
	char message_received[255] = ""; //without CRC
	sprintf(message_received,"%s",raw_msg_received.c_str());
	char *msg;
	memcpy(msg,message_received,strlen(message_received));
	//cout << "MessageToCRC: "<<msg<< "Size: "<<strlen(msg)<<endl;
	int c = _compute_CRC(msg);
	//cout << "msg_received with calculated CRC: "<<msg<<endl;
	return 0 == strcmp(msg,data);
}


int Serial_Port::_read_port_server(char *data, int size)
{

  int n;
	bool STOP=false;
	char buf[255];
	int count = 0;
	double read_time = 0;
	clock_t t_ini;
	fd_set fds;
  struct timeval timeout;
	int ret = 0;

	pthread_mutex_lock(&lock_s);

	 FD_ZERO(&fds);
   FD_SET (fd, &fds);
    
   timeout.tv_sec = 0; //seconds
   timeout.tv_usec = 0; //microseconds
		
   ret=select (FD_SETSIZE,&fds, NULL, NULL,&timeout);

	while (!STOP && ret > 0)
	{
		
		n = read(fd,buf,255);
	  buf[n]=0;             // envio de fin de cadena, a fin de poder usar printf 
	  //printf("%s",buf);
		count += n;
		string msg = string(buf);
		strcat(data,buf);
	  if (msg.find('\n')!=string::npos) STOP=true;

	}

	pthread_mutex_unlock(&lock_s);	
	
	//check crc -- Provoca un Segmentation faultS, seguramente por la memcpy
	/*if (0 < count)
		if (!_check_CRC(data)){
			wrong_packets_counter++;
		}*/


	/*fd_set fds;
  struct timeval timeout;
  int count=0;
  int ret;
  int n;
	
	pthread_mutex_lock(&lock_s);

  do {
    FD_ZERO(&fds);
    FD_SET (fd, &fds);
    
    timeout.tv_sec = 0; //seconds
    timeout.tv_usec = 1; //microseconds
		
   	ret=select (FD_SETSIZE,&fds, NULL, NULL,&timeout);
    
    if (ret==1) {
      n=read (fd, &data[count], size-count); 
      count+=n;
      data[count]=0;
			cout << "Data received: "<<data;
    }
  } while (count<size && ret==1);


	//tcflush(fd, TCIOFLUSH);

		
	//tcflush(fd, TCIFLUSH);

	pthread_mutex_unlock(&lock_s);	*/

	/*pthread_mutex_lock(&lock_s);

	int count = recv_to(data,size,0,0);
	pthread_mutex_unlock(&lock_s);*/
	
  return count;
}


int Serial_Port::_write_port_server(char *data, int size)
{
	pthread_mutex_lock(&lock_s);

	int bytesWritten = 0;
	
	//int crc = _compute_CRC(data);

	//Showing the end of the string sended
	//tcflush(fd, TCOFLUSH);

	bytesWritten = write(fd, data, strlen(data));

 	tcdrain(fd);

	//data = NULL;
	

	pthread_mutex_unlock(&lock_s);	

	return bytesWritten;
}


