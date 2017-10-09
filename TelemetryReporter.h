#ifndef TelemetryReporter_H
#define TelemetryReporter_H

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <vector>
#include <pthread.h>
#include "Serial_Port.h"
#include "ServerSelect.h"

using namespace std;

void* handler_thread(void *args);


class TelemetryReporter {

	public:

		TelemetryReporter();
		/*Constructor with RF - Type_Conexion = 2*/
		TelemetryReporter(ServerSelect *server, Serial_Port *serial, int frecuency, int t);

		/*Destructor*/
		~TelemetryReporter();

		/*Execute the control algorithm*/
		void handler();
		int start();
		int stop(); 

		vector<float> getRpiTM();

		void setApmTM(vector<float> tm);

		void sendTM(vector<float> rpitm, vector<float> apmtm);


	private:

		vector<float> apmTM_DATA;
		/**/
		bool time_to_exit;

		unsigned int frecuency;
		int connection_type;

		/*Serial Connection with OGR-GOAC via RF - connection_type = 2*/
		Serial_Port *serial_OGR;
		string uart_OGR;
		int baud_uart_OGR;	

		/*Server TCP/IP - connection_type = 1*/
		ServerSelect *server_OGR;

		pthread_t handler_tid;


		
};

#endif
