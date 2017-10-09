#include <stdlib.h>
#include <sstream>
#include <unistd.h>
#include "Drone_Interface.h"
#include "APM_Interface.h"


#define M_LENGTH 800

using namespace std;

/*Internal Layer which create an Interface to control the UAV via MAVLink Protocol*/
APM_Interface apm_interface;

/*Constructor*/
Drone_Interface::Drone_Interface (bool sim, int port, string uart, int baud, string s_uart, int b_uart, int type_c) {

	simulation = sim;
	port_server = port;
	uart_APM = uart;
	baud_uart_APM = baud;
	uart_OGR = s_uart;
	baud_uart_OGR = b_uart;
	type_conexion = type_c;
	time_to_exit = false;
	current_action=NULL;
}

/*Destructor*/
Drone_Interface::~Drone_Interface() {
		tm_reporter.stop();
	apm_interface.stop();
	if (0 != serial_APM.close_serial())
		cout << "[UAV Server] - We had a problem closing the serial port "<<uart_APM<<endl;
	if (0 != serial_OGR.close_serial())
		cout << "[UAV Server] - We had a problem closing the serial port "<<uart_OGR<<endl;
}


void Drone_Interface::start() {
	bool ogr_linked = false;

	/*Simulation mode*/
	if (simulation)
			cout<<"[UAV Server] - Simulation mode enabled"<<endl;
		else
			cout<<"[UAV Server] - Simulation mode disabled"<<endl;

 /*Openning the communication with OGR*/
	switch(type_conexion){
		case 1: /*Option 1: Starting TCP/IP Server */
			if(!server.open(port_server))
  			cout << "[UAV Server] - Cannot open TCP/IP Server" << endl;
  		else{
				cout << "[UAV Server] - TCP/IP Server opened successfully. Listening in "<< port_server << endl;
				ogr_linked = true;
				/*TelemetryReporter*/		
				tm_reporter = TelemetryReporter(&server, NULL ,1,1);
			}
			break;
		case 2: /*Option 2: Openning the RF Serial Port*/
			char uart[M_LENGTH];
			sprintf(uart,"%s",uart_OGR.c_str());
			serial_OGR = Serial_Port(uart,baud_uart_OGR);

			if (0 != serial_OGR.open_serial())
				cout << "[UAV Server] - We had a problem openning the RF Serial Port "<<uart_OGR<<endl;
			else{
				ogr_linked = true;	
				cout << "[UAV Server] - Linked to OGR successfully. RF Serial Port "<<uart_OGR<<" openned at "<<baud_uart_OGR<<" bauds"<<endl;
				/*TelemetryReporter*/			
				tm_reporter = TelemetryReporter(NULL ,&serial_OGR,1,2);	
			}
			break;

	}

	if(ogr_linked){
			
			/*Start Serial Port to connect via MavLink*/
			if (!simulation){
				char uart[M_LENGTH];
				sprintf(uart,"%s",uart_APM.c_str());
				serial_APM = Serial_Port(uart,baud_uart_APM);

				/*Start Telemetry Reporter*/
				if (0 > tm_reporter.start())
					cout << "[UAV Server] - We had a problem launching the telemetry reporter"<<endl;
				else
					cout << "[UAV Server] - Telemetry Reporter launched successfully"<<endl;
			
				if (0 != serial_APM.open_serial())
					cout << "[UAV Server] - We had a problem openning the Serial Port "<<uart<<endl;
				else
					cout << "[UAV Server] - Linked to APM successfully. Serial Port "<<uart<<" openned at "<<baud_uart_APM<<" bauds"<<endl;
				
				/*Start APM communication interface*/
				apm_interface = APM_Interface(this,&serial_APM);
				apm_interface.lastStatus.packet_rx_drop_count = 0;

				if (0 != apm_interface.start()){
					cout<< "[UAV Server] - We had a problem launching the APM Interface"<<endl;
				}
				else
					cout<< "[UAV Server] - Internal layer APM launched successfully"<<endl;
			}

			/*Launch manager handler*/
			handler();

		/*Initialize the counter of actions*/
		id_action = 0;
	}

}

void Drone_Interface::stop() {
	time_to_exit = true;
}


/* Read new messages from OGR */
void Drone_Interface::handler(){

	char message[M_LENGTH];
	int read = 0;
	
		while (!time_to_exit){
			
			/*Waiting the message TC Telemetry from the OGR-GOAC. 
				Both connections are blocked, it means that the thread is blocked until receive a message.*/

			/*Updating APM Telemetry*/
			tm_reporter.setApmTM(apm_interface.getApmTelemetry());
			
			
			if (NULL==current_action){
				
				if (type_conexion==1){
					//Reading message from a client TCP/IP - BLOCKING
					read = server.receive(message);
				}else if (type_conexion==2){
					//Reading message from a client Serial
					read = serial_OGR._read_port_server(message,M_LENGTH);
				}
				
				/*Analyze the received data and store in the input buffer (buffer_In)*/
				if (read>1){
					string s_msg = string(message); 
				
					if (s_msg.find("CMD")!=string::npos){
						//cout << "Length: "<<s_msg.length()<<endl;
						s_msg = s_msg.substr(s_msg.find("CMD"),s_msg.length());
						cout << "\n[UAV Server] - Received DATA: "<<s_msg<<endl;

						/*Convert cmd to token*/
						pthread_mutex_lock(&lock_action);

						current_action = createToken(s_msg);

						if (0 != pthread_create (&dispatcher_tid, NULL, &dispatcher_thread, this)){
							cout<< "[UAV Server] - Error: It was an error executing the dispatcher"<<endl;
						}
						
						pthread_cond_signal(&cv_action);
    				pthread_mutex_unlock(&lock_action);
						
						/*Simulate the UAV responses*/
						simulation_mode();
						
					}
				}

				/*Resets*/			
				read = 0;
				message[0] = 0;
	
			}

		sleep(HANDLER_FREQ);
	}
}

void *dispatcher_thread(void *args){

	Drone_Interface *Drone_interface = (Drone_Interface *) args;
	//
	Drone_interface->dispatcher();	
	//
	return NULL;
}

void Drone_Interface::dispatcher() {

	//while (!time_to_exit){

		pthread_mutex_lock(&lock_reached);

		while (!current_action->reached)
		{
			pthread_cond_wait(&cv_reached, &lock_reached);
		}

	
		/*Prepare the Observation REPLY*/
		if (current_action->getPredicate()=="TakingOff"){
			current_action->setPredicate("InFlight");

		}else if (current_action->getPredicate()=="FlyingTo"){
			current_action->setPredicate("At");

		}else if (current_action->getPredicate()=="Landing"){
			current_action->setPredicate("OnLand");

		}else if (current_action->getPredicate()=="TakingPicture"){
			current_action->setPredicate("CamIdle");
			current_action->attributes.clear();

		}else if (current_action->getPredicate()=="Checking"){
			current_action->setPredicate("Checked");

		}else if (current_action->getPredicate()=="ArmingMotors"){
			current_action->setPredicate("MotorsArmed");

		}else if (current_action->getPredicate()=="DisarmingMotors"){
			current_action->setPredicate("Idle");
		}

		/*Send Response via TM Telemetry*/
		sendTM(current_action->generateMsg());

		current_action = NULL;

		pthread_mutex_unlock(&lock_reached);
		
	//}

}

void Drone_Interface::simulation_mode(){
	if (simulation){
			/*Prepare the simulated observation response*/
			if (current_action->getPredicate()=="TakingOff"){
				current_action->setPredicate("InFlight");

			}else if (current_action->getPredicate()=="FlyingTo"){
				current_action->setPredicate("At");

			}else if (current_action->getPredicate()=="Landing"){
				current_action->setPredicate("OnLand");

			}else if (current_action->getPredicate()=="TakingPicture"){
				current_action->setPredicate("CamIdle");
				current_action->attributes.clear();

			}else if (current_action->getPredicate()=="Checking"){
				current_action->setPredicate("Checked");
			}else if (current_action->getPredicate()=="ArmingMotors"){
				current_action->setPredicate("MotorsArmed");
			}else if (current_action->getPredicate()=="DisarmingMotors"){
				current_action->setPredicate("Idle");
			}

			/*Simulating a delay of an action accomplish by the robotic system*/
			unsigned int seconds = 2;
			sleep(seconds);

			/*Send Response via TM Telemetry*/
			sendTM(current_action->generateMsg());

			current_action = NULL;

	}
}


void Drone_Interface::sendTM(string tm){
	
			char message[M_LENGTH];
			message[0] = 0;
			if (tm!=""){
			sprintf(message,"%s",tm.c_str());

			if (type_conexion==1){
					if (server.sendData(message)>0){
						cout << "[UAV Server] - Send DATA: "<<message<<endl;
					}
			}else if (type_conexion==2){
					serial_OGR._write_port_server(message,M_LENGTH);
					cout << "[UAV Server] - Send DATA: "<<message<<endl;					
				}
			}

}

TokenTC* Drone_Interface::createToken(string msg) {
	TokenTC *tk = new TokenTC(id_action++);
	int cnt=0;
	std::size_t pos = msg.find("|");
	string att = "";

	while (pos!=std::string::npos){
		switch(cnt){
			case 0:
				tk->setType(msg.substr(0,pos));
				break;
			case 1:
				tk->setTick(atoi(msg.substr(0,pos).c_str()));
				break;
			case 2:
				tk->setTM(msg.substr(0,pos));
				break;
			case 3:
				tk->setPredicate(msg.substr(0,pos));
				break;
			default:
				if (cnt%2==0){
					att=msg.substr(0,pos); //Attribute Name
				}else { //Attribute Value
					string::size_type sz;     
					int v = atoi(msg.c_str());
					tk->attributes.push_back(Attribute(att,v));
				}
				break;
		}	

		msg = msg.substr(pos+1);
		pos = msg.find("|");
		cnt++;
	}
	return tk;
}


TokenTC* Drone_Interface::get_current_action(){

	pthread_mutex_lock(&lock_action);
	while (NULL==current_action)
	{
		pthread_cond_wait(&cv_action, &lock_action);
	}
	pthread_mutex_unlock(&lock_action);
	return current_action;
}

void Drone_Interface::set_current_action_reached(){

	pthread_mutex_lock(&lock_reached);

	current_action->reached = true;
						
	pthread_cond_signal(&cv_reached);
	pthread_mutex_unlock(&lock_reached);

}
