#include <cstdlib>
#include <iostream>
#include <csignal>
#include "Drone_Interface.h"

using namespace std;


/*This is the main file that launch the functional layer to control the UAV. It's executed on the Raspberry Pi 3 onboard the UAV.
	In summary, it launchs a server which manages two connections, an EXTERNAL connection to communicate with the OGR (GOAC) via RF and another INTERNAL connection to communicate with the Mini APM 
  via Serial Port. The external connection can be via TCP/IP or via RF. The internal connection always is via Serial Port.*/
int main(int argc, char** argv) 
{
	/*External connection type by default*/
	int type_conexion = 2; 

	/*External Connection type = 1 -> Connection TCP/IP to OGR*/
	int server_port = 50505;

	/*External Connection Type = 2 -> Connection RF (via Serial Port) to OGR*/
	int server_baud = 57600; 
	string server_uart = "/dev/ttyUSB0";

	/*Simulation mode DISABLED by default. True = Enabled ; False = Disabled*/
	bool simulation = false; 
	
	/*Internal Serial Connection */
  int baud_apm = 115200;
	string uart_apm = "/dev/ttyACM0";
  
	/****																																																																					
				Format : serverAPM (optional)-S (option 1) -p PORT_SERVER (option 2) -su SERVER_UART -sb SERVER_BAUD (mandatory) -u UART_APM -b BAUD_APM
																																																																									****/

	if (argc < 4 || argc > 10){
		cout << "Usage:"<<endl;
		cout<< "	serverAPM [optional] -S (option 1) -p PORT_SERVER (option 2) -su SERVER_UART -sb BAUD_SERVER [mandatory] -u UART_APM -b BAUD_APM"<<endl<<endl;
		cout<< "   -S: Is an [optional] parameter to enable or disable the simulation mode. If you introduce this parameter into your execution command you will enable the simulation."<<endl;
	  cout<< "   -p: (*Option 1) Is a [mandatory] parameter to indicate the port on which the server listens."<<endl;
		cout<< "   -su: (*Option 2) Is a [mandatory] parameter to indicate the uart on which the server will connect with OGR"<<endl;
		cout<< "   -sb: (*Option 2) Is a [mandatory] parameter to indicate the baudrate of the uart on which the server will connect with OGR"<<endl;
		cout<< "   -u: Is a [mandatory] parameter to indicate the uart port on which the interface will connect with the APM in the Drone"<<endl;
		cout<< "   -b: Is a [mandatory] parameter to indicate the baudrate of the UART indicate."<<endl<<endl;
		cout<< " We have to options to connect with OGR, and it is mandatory include one of them in the execution "<<endl;
		cout<< "  *Option 1: This option will connect through TCP/IP"<<endl;
		cout<< "  *Option 2: This option will connect through Serial"<<endl;
	}else {
		int cnt = 1;
		while (cnt<10){
			stringstream cmd;
			cmd << argv[cnt];

			if (cmd.str()=="-S"){
				simulation = true;
			}else if (cmd.str()=="-p"){
				type_conexion = 1;
				if (atoi(argv[cnt+1]) > 2 && atoi(argv[cnt+1]) < 65535){
					server_port = atoi(argv[cnt+1]);
				}else
					cout << "[UAV Server] -> Error: Port "<<atoi(argv[cnt+1])<<" incorrect. Please, select a port between 2 and 65535."<<endl;
			}else if (cmd.str()=="-u"){
				uart_apm = argv[cnt+1];
			}else if (cmd.str()=="-b"){
				baud_apm = atoi(argv[cnt+1]);
			}else if (cmd.str()=="-su"){
				type_conexion = 2;
				server_uart = argv[cnt+1];
			}else if (cmd.str()=="-sb"){
				server_baud = atoi(argv[cnt+1]);
			}

		cnt++;
		}

		/*Launch the functional layer with the user's parameters*/
		Drone_Interface* d_link = new Drone_Interface(simulation,server_port,uart_apm,baud_apm,server_uart,server_baud, type_conexion);
		d_link->start();
	}
	
    return 0;
}
