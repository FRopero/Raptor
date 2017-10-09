#include "TelemetryReporter.h"

#define M_LENGTH 800

using namespace std;

TelemetryReporter::TelemetryReporter() {

	serial_OGR = NULL;
	server_OGR = NULL;
	time_to_exit = false;
	frecuency = -1;
	connection_type = -1;

}


TelemetryReporter::TelemetryReporter(ServerSelect *server, Serial_Port *serial,  int f, int t){

	if (t==2)
		serial_OGR = serial;

	if (t==1)
		server_OGR = server;

	time_to_exit = false;
	frecuency = f;
	connection_type = t;
}


TelemetryReporter::~TelemetryReporter() {}

void *handler_thread(void *args){

	TelemetryReporter *TMReporter = (TelemetryReporter *) args;
	
	TMReporter->handler();	
	
	return NULL;
}

int TelemetryReporter::start(){

	if (0 != pthread_create (&handler_tid, NULL, &handler_thread, this)){
		return -1;
	}

	return 0;
}

int TelemetryReporter::stop(){
	time_to_exit = true;
	pthread_join(handler_tid,NULL);
	pthread_exit(NULL);
}

void TelemetryReporter::handler() {
	vector<float> rpiTM;
	char message;

	while (!time_to_exit)
	{
		
		message = 0;

		//PS FORMAT: STATUS|cpu|mem|rss|cputime|
		rpiTM = getRpiTM();
		//FORMAT: battery_voltage|gps_fix|cpu_apm

		sendTM(rpiTM, apmTM_DATA);

		//Resets
		rpiTM.assign(rpiTM.size(),0);
		apmTM_DATA.assign(apmTM_DATA.size(),0);

		sleep(frecuency);
	}

}

void TelemetryReporter::sendTM(vector<float> rpitm, vector<float> apmtm) {

	int r, a = 0;
	char message[M_LENGTH];
	char apmTM[M_LENGTH];
	
	message[0] = 0;
	apmTM[0]	= 0;
	
	if ((rpitm.size() > 0) && (0 < rpitm.at(1)))
		r = snprintf(message, M_LENGTH, "STATUS|%.2f|%.2f|%.2f|%.2f|",rpitm.at(0),rpitm.at(1),rpitm.at(2),rpitm.at(3));		//STATUS|cpu|mem|rss|cputime|
	
	if ((apmtm.size() > 0) && (0 < apmtm.at(1))) //to ensure not sending 0s
	{
		a = snprintf(apmTM,M_LENGTH,"%.2f|%.2f|%.2f|",apmtm.at(0),apmtm.at(1),apmtm.at(2)); //lipo_battery_voltage|gps_status|cpu_apm
		strcat(message,apmTM);
	}
	
	if (0 < r){

		if (connection_type==1){
				if (server_OGR->sendData(message)>0){
					//cout << "[UAV Server] - Send DATA: "<<message<<endl;
				}
		}else if (connection_type==2){
				serial_OGR->_write_port_server(message,M_LENGTH);
				//cout << "[UAV Server] - Send DATA: "<<message<<endl;					
			}
	}

}


void TelemetryReporter::setApmTM(vector<float> tm) {
	apmTM_DATA.assign(tm.size(),0);
	
	for(int i=0;i<=tm.size()-1;i++)
	{
		apmTM_DATA.at(i) = tm.at(i);
	}
	
}

vector<float> TelemetryReporter::getRpiTM() {

	char pid[128];
	system ( "/bin/pidof ./Drone_Interface > log.txt");

	ifstream fe("log.txt");

   while(!fe.eof()) {
      fe >> pid;
   }
   fe.close();

	//PS OUTPUT: %CPU %MEM Memory(KiB) CpuTime("[DD-]hh:mm:ss")
	char cmdline[256] = "/bin/ps ho pcpu:1,pmem:1,rss:1,cputime ";


	strcat(cmdline, pid);
	strcat(cmdline, " > log.txt");
	system (cmdline);

	string data[4];
	int r=0;
	ifstream fes("log.txt");
   while(!fes.eof()) {
      fes >> data[r];
			r++;
   }
   fes.close();


	vector<float> ret;
	char tmp[128];
	ret.assign(4,0);
	for (int i=0; i<4; i++){
		sprintf(tmp,"%s",data[i].c_str());
		ret.at(i) = atof(tmp);
		tmp[0] = 0;
	}

	
	 std::size_t found = data[3].find("-");
  if (found!=std::string::npos)
  {
			char days[128];
			sprintf(days,"%s",data[3].substr(0, 2).c_str()); 
      ret.at(3) = 86400 * atoi(days);
      data[3] = data[3].substr(3);
  }
	char dat[256];
	sprintf(dat,"%s",data[3].c_str());
  char * pch = strtok(dat,":");
	float tokens [3] = {0,0,0};
	int d = 0;
	while ( pch != NULL)
	{
		tokens[d] = atoi(pch);
    pch = strtok (NULL, ":");
		d++;
	}
  ret.at(3) += 3600 * tokens[0] + 60 * tokens[1] + tokens[2] ;

	return ret;

}
