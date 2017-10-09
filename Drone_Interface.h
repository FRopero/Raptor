#ifndef Drone_Interface_H
#define Drone_Interface_H

#include <iostream>
#include <pthread.h>
#include <vector>
#include "TokenTC.h"
#include "Serial_Port.h"
#include "ServerSelect.h"
#include "TelemetryReporter.h"

#define HANDLER_FREQ 0.5 //secs

using namespace std;

void* dispatcher_thread(void *args);

class Drone_Interface {

	public:
		/*Constructor of the thread*/
		Drone_Interface(bool sim, int port, string uart, int baud, string server_uart, int baud_uart, int type_conexion);

		/*Destructor of the thread*/
		~Drone_Interface();

		/*Send TM Telemetry message from the output buffer*/
		void sendTM(string tm);

		/*Translate a TC Telemetry message into an object with attributes*/
		TokenTC* createToken(string msg);

		/*Initiate the EXTERNAL communication with OGR-GOAC and the INTERNAL communication with Mini APM*/		
		void start();
		void stop();
		
		/*Execute the control algorithm*/
		void handler();
		void dispatcher();

		/*External method used by APM_Interface. Look into the actions list an action not executed yet. Then, this action is going to execute.*/
		TokenTC* get_current_action();
		
		/*External method used by APM_Interface. Look into the actions list if the action ID has been reached or completed*/
		void set_current_action_reached(); 

		void simulation_mode();

		pthread_mutex_t lock_action = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cv_action = PTHREAD_COND_INITIALIZER;

		pthread_mutex_t lock_reached = PTHREAD_MUTEX_INITIALIZER;
		pthread_cond_t cv_reached = PTHREAD_COND_INITIALIZER;
		

	private:

		/**/
		pthread_t dispatcher_tid;

		/*Flag to allow us finishing the interface and their threads*/
		bool time_to_exit;

		/*Serial Connection with Mini APM*/
		Serial_Port serial_APM; 
		string uart_APM;
		int baud_uart_APM;

		/*Serial Connection with OGR-GOAC via RF*/
		Serial_Port serial_OGR;
		string uart_OGR;
		int baud_uart_OGR;	

		/*TCP/IP Connection with OGR-GOAC*/
		int port_server;

		/*Connection type between OGR and Drone*/
		int type_conexion; // 1=TCP/IP ; 2=Serie
		
		/*Simulation Mode*/
		bool simulation;

		/*Server TCP/IP*/
		ServerSelect server;

		int id_action; //Counter of the actions received from OGR

		TokenTC *current_action;

		TelemetryReporter tm_reporter;
		
};

#endif
