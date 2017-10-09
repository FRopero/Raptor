#include <stdlib.h>
#include <unistd.h>
#include "APM_Interface.h"

#include "mavlink/v1.0/ardupilotmega/ardupilotmega.h"


/*****  Procedimiento para despegar, volar y aterrizar
   		1. Guided Mode - SET_MODE / MAV_CMD_DO_SET_MODE (cmd_long)
			2. Check hardware pre-vuelo - MAV_CMD_PREFLIGHT_CALIBRATION (cmd_long)
			2. Armar motores - MAV_CMD_COMPONENT_ARM_DISARM (cmd_long)
			3. Enviar comando Take Off - MAV_CMD_NAV_TAKEOFF (cmd_long / mission_item) 
			4. Sobrescribir canales RC y aumentar acelerador levemente ¿Es necesario?
			5. Enviar comandos waypoints - MAV_CMD_NAV_WAYPOINT (mission_item)
			6. Enviar comando Land - MAV_CMD_NAV_LAND (cmd_long / mission_item)
			7. Sobreescribir canales RC y poner el acelerador a 0 ¿Es necesario?
			8. Desarmar motores - MAV_CMD_COMPONENT_ARM_DISARM (cmd_long)
			
******/

using namespace std;

APM_Interface::APM_Interface(){

	vehicle_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	gcs_id = 255; // companion computer component id

	mavlink_data.sysid  = vehicle_id;
	mavlink_data.compid = autopilot_id;

	mavlink_data.command_ack.command = -1;
	mavlink_data.mission_item_reached.seq = 0;
	mavlink_data.mission_request.seq = -1;
	mavlink_data.sys_status.voltage_battery = 0;
	mavlink_data.gps_raw_int.fix_type = 0;
  list_seq = 0;
	time_to_exit   = false;
	
	serial_port = NULL;
	mission_item_seq = 0;
	mission_ack = false;
	item_request = false;
	waypoint_current = false;
	attempts_takeoff = 0;
	attempts_flyingto = 0;
	failsafe = false;

	home_location.longitud = 0;
	home_location.latitud = 0;
	home_location.altitud = 0;

}

APM_Interface::APM_Interface(Drone_Interface *ogr, Serial_Port *sp){

	vehicle_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	gcs_id = 255; // companion computer component id

	mavlink_data.sysid  = vehicle_id;
	mavlink_data.compid = autopilot_id;

	mavlink_data.command_ack.command = -1;
	mavlink_data.mission_item_reached.seq = 0;
	mavlink_data.mission_request.seq = -1;
	mavlink_data.sys_status.voltage_battery = 0;
	mavlink_data.gps_raw_int.fix_type = 0;

	list_seq = 0;
	time_to_exit   = false;

	serial_port = sp;
	Drone_interface = ogr;
	mission_item_seq = 0;
	mission_ack = false;
	item_request = false;
	waypoint_current = false;
	attempts_takeoff = 0;
	attempts_flyingto = 0;
	failsafe = false;

	home_location.longitud = 0;
	home_location.latitud = 0;
	home_location.altitud = 0;

}

APM_Interface::~APM_Interface(){
}

uint64_t get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

int APM_Interface::start(){

	/*Check status serial port*/
	if ( 1 != serial_port->status ) {
		return -1;
	}
	/*Reading thread*/
	if (0 != pthread_create (&read_tid, NULL, &read_thread, this)){
		return -1;
	}
	/*Setup APM with some initial parameters*/
	if (0 != setup_APM()){

		cout << "[APM Interface] - Failure on APM Setup"<<endl;
		return -1;
	}else{
		/*Dispatcher thread*/
		if (0 != pthread_create( &dispatch_tid, NULL, &dispatch_thread, this)){
			return -1;
		}
}

	return 0;

}

int APM_Interface::setup_APM(){
	int attempts = 0;

	/*cout<<"[APM Interface] - Opening Camera "<<endl;
 	if ( !Camera.open()) {
		cout<<"[APM Interface] - Error opening camera"<<endl;
		return -1;
	}*/


	while ( not mavlink_data.sysid )
	{
		if ( time_to_exit )
			return -1;
		usleep(500000); // check at 2Hz
	}

	/*System ID*/
	if ( not vehicle_id )
	{
		vehicle_id = mavlink_data.sysid;
		cout << "[APM Interface] - Vehicle System Id: "<<vehicle_id<<endl;
	}else
		return -1;

	/*Component ID*/
	if ( not autopilot_id )
	{
		autopilot_id = mavlink_data.compid;
		cout << "[APM Interface] - APM Component Id: "<<autopilot_id<<endl;
	}else
		return -1;

	if (attempts==CONNECT_ATTEMPTS){
		cout << "[APM Interface] - Couldn't connect with the drone"<<endl;
		return -1;
	}

	/*Enabling telemetry*/

	mavlink_message_t message;
	mavlink_request_data_stream_t msg_request;
	msg_request.req_message_rate = 10000;
	msg_request.target_system = vehicle_id;
	msg_request.target_component = autopilot_id;
	msg_request.req_stream_id=0;
	msg_request.start_stop=1; //1 = start sending - 0 = stop sending
	mavlink_msg_request_data_stream_encode(gcs_id,gcs_id,&message,&msg_request);
	int len = write_message(message);

	sleep(INIT_SEC);

	/*Cleaning previous mission set up*/
	if (set_flightmode(STABILIZE)){

		sleep(INIT_SEC);

		if (do_clear_mission()){
		
				/*Verifying critical systems*/
				if ((len > 1)&&(verifying_critical_systems())){

					return 0;
				}
		}
	}
	cout << "[APM Interface] - We have experimented a problem setting up the drone"<<endl;
	return -1;
	
}

bool APM_Interface::verifying_critical_systems() {
	/*List of critical systems so far
		1. Battery
		2. GPS
		3. Communication
	*/

		/* 1. Battery*/
		pthread_mutex_lock(&lock_battery);
		while (0 == mavlink_data.sys_status.voltage_battery)
		{
			pthread_cond_wait(&cv_battery, &lock_battery);
		}
			
		if (START_FLIGHT_BATT > mavlink_data.sys_status.voltage_battery/1000.0f)
		{
			cout << "[APM Interface] - Battery level [ "<< mavlink_data.sys_status.voltage_battery/1000.0f <<"V ] too low"<<endl;
			return false;
		}
		else
		{
			cout << "[APM Interface] - Battery level [ "<< mavlink_data.sys_status.voltage_battery/1000.0f <<"V ] suitable"<<endl;
		}
		pthread_mutex_unlock(&lock_battery);
		
		/* 2. GPS   3==3D-Fix*/
		double gps_fix_time = 0; // in seconds
		clock_t t_ini;

		pthread_mutex_lock(&lock_gps);
		if (3 > mavlink_data.gps_raw_int.fix_type )
		{
			cout << "[APM Interface] - GPS Fix too low" << endl;
			cout << "[APM Interface] - Waiting to a 3D signal of the GPS ..."<<endl;
		}
		t_ini = clock();
		gps_fix_time = (double)(clock()-t_ini) / CLOCKS_PER_SEC;

		
		while (3 != mavlink_data.gps_raw_int.fix_type && gps_fix_time < GPS_FIXING_TIME)
		{
			gps_fix_time = (double)(clock()-t_ini) / CLOCKS_PER_SEC;
			pthread_cond_wait(&cv_gps, &lock_gps);
		}
		pthread_mutex_unlock(&lock_gps);

		if (gps_fix_time > GPS_FIXING_TIME){
			cout << "[APM Interface] - GPS Fix too low" << endl;
			return false;
		}
		if (3 == mavlink_data.gps_raw_int.fix_type)
			cout << "[APM Interface] - GPS Fix [3D] suitable" <<endl;

		cout << "[APM Interface] - GPS Fix [3D] suitable" <<endl;

	return true;
}

int APM_Interface::stop() {
	time_to_exit = true;
	//Join de los thread read and write
	pthread_join(read_tid,NULL);
	pthread_join(dispatch_tid,NULL);
	pthread_exit(NULL);
}

void APM_Interface::dispatcher(){
	TokenTC* tk = NULL;
	TokenTC* last_exec_tk = NULL; //To avoid execute the same action that has been reached a tick before, but the GCS hasn't send the next current_action
	while ( !time_to_exit )
	{	
		if (!failsafe){
			tk = Drone_interface->get_current_action();
		}//else
			//failsafe_handler(tk);

		if (tk!=NULL && tk!=last_exec_tk){ //Execute the action only if it isn't NULL or equal to the last action.
			if (token_execution_handler(tk)){ //Handling the TokenTC execution 
				if (token_response_handler(tk)){ //Handling the TokenTC response
					Drone_interface->set_current_action_reached();
					last_exec_tk = tk;
					cout << "[APM Interface] - Action '"<<tk->getPredicate()<<"' ID="<<tk->id<<" executed in APM "<<endl;				
				}else {
					/*La acción se ha ejecutado pero ha pasado un tiempo y el sistema no ha alcanzado el objetivo, enviar señal de failsafe al planificador para que delibere las contramedidas necesarias*/
					cout << "[APM Interface] - The system was not be able to reach the goal '"<<tk->getPredicate()<<"' "<<endl;
					
					cout << "[APM Interface] - Goal reached fail: Failsafe ENABLED"<<endl;
					tk = NULL;
					failsafe = true;
				}
			}else {
				cout << "[APM Interface] - The '"<<tk->getPredicate()<<"' action could not be executed"<<endl;

				/*Si se produce un fallo al ejecutar cualquier acción del nivel superior, enviar señal de failsafe al planificador para que delibere las contramedidas necesarias*/
				cout << "[APM Interface] - Goal execution fail: Failsafe ENABLED"<<endl;
				tk = NULL;
				failsafe = true;
			}
		}
		
		usleep(100); //100 microsecs = 0.0001 seg - 10000 Hz
	}
	
}

bool APM_Interface::token_response_handler(TokenTC* tk){

	if ("TakingOff"==tk->getPredicate()){

		return wait_reached_takeoff();
	}else if ("FlyingTo"==tk->getPredicate()){
		
		return wait_reached_flyto();
	}else if("Landing"==tk->getPredicate()){
		
		return wait_reached_land();
	}else if("TakingPicture"==tk->getPredicate()){

		return true;//Its reached in the instant it was sended
	}else if ("Checking"==tk->getPredicate()) {
		
		return true;//Its reached in the instant it was sended
	}else if ("ArmingMotors"==tk->getPredicate()) {

		return true;//Its reached in the instant it was sended
	}else if ("DisarmingMotors"==tk->getPredicate()) {

		return true;//Its reached in the instant it was sended
	}
	return true;

}

bool APM_Interface::wait_reached_takeoff(){

	double execution_time = 0; // in seconds
	clock_t t_ini;

	t_ini = clock();
	execution_time = (double)(clock()-t_ini) / CLOCKS_PER_SEC;
	while (mavlink_data.mission_item_reached.seq != waypoint_seq && execution_time < ELAPSED_TIME_TAKEOFF)
	{
		execution_time = (double)(clock()-t_ini) / CLOCKS_PER_SEC;
	}
	return (mavlink_data.mission_item_reached.seq == waypoint_seq);

}

bool APM_Interface::wait_reached_flyto(){
	
	double execution_time = 0; // in seconds
	clock_t t_ini;

	t_ini = clock();
	execution_time = (double)(clock()-t_ini) / CLOCKS_PER_SEC;
	while (mavlink_data.mission_item_reached.seq != waypoint_seq && execution_time < ELAPSED_TIME_FLYINGTO)
	{
		execution_time = (double)(clock()-t_ini) / CLOCKS_PER_SEC;
	}
	return (mavlink_data.mission_item_reached.seq == waypoint_seq);

}

bool APM_Interface::wait_reached_land(){
	
	double execution_time = 0; // in seconds
	clock_t t_ini;

	t_ini = clock();
	execution_time = (double)(clock()-t_ini) / CLOCKS_PER_SEC;
	while (mavlink_data.mission_item_reached.seq != waypoint_seq && execution_time < ELAPSED_TIME_LANDING)
	{
		execution_time = (double)(clock()-t_ini) / CLOCKS_PER_SEC;
	}
	return (mavlink_data.mission_item_reached.seq == waypoint_seq);

}

void *dispatch_thread(void *args){

	APM_Interface *APM_interface = (APM_Interface *)args;

	APM_interface->dispatcher();

	return NULL;

}


void *read_thread(void *args){

	APM_Interface *APM_interface = (APM_Interface *) args;
	
	APM_interface->reader();	

	return NULL;
}


void APM_Interface::reader(){
/* Read new messages from the APM and save them in mavlink_data */
		while (!time_to_exit){
			read_messages();
			usleep(100000); //100000 microseg = 0.1 seg - 10 Hz
		}
}



/*Launch to execution the required action. The return TRUE it means that this action is executing at this moment.*/
bool APM_Interface::token_execution_handler(TokenTC* tk){

	if ("TakingOff" == tk->getPredicate()){
		
		do{
			//Get Home location (altitud) 
			update_home_location();

			if (attempts_takeoff < AT_TAKEOFF){
				attempts_takeoff++;
			}else
				return false;
		}while (!do_takeoff(tk));

	}else if ("FlyingTo" == tk->getPredicate()){
		
		do{
			//Get Home location
			update_home_location();

			if (attempts_flyingto < AT_FLYINGTO){
				attempts_flyingto++;
			}else
				return false;
		}while (!do_flyingto(tk));

	}else if("Landing" == tk->getPredicate()){

		do{
			//Get Home location
			update_home_location();

			if (attempts_landingto < AT_LAND){
				attempts_landingto++;
			}else
				return false;
		}while (!do_landingto(tk));

	}else if("TakingPicture"==tk->getPredicate()){
		return do_takepict();
	
	}else if ("Checking"==tk->getPredicate()) {

		return do_precheck();
	}else if ("ArmingMotors"==tk->getPredicate()) {

		return do_arm_disarm(1);
	}else if ("DisarmingMotors"==tk->getPredicate()) {

		return do_arm_disarm(0);
	}

	return true;
}

bool APM_Interface::do_precheck(){

	if (command_long_send(MAV_CMD_PREFLIGHT_CALIBRATION,0,0,1,0,0,0,-1))
		return true;
	
  cout<<"APM Interface] - do_precheck experimented a problem on MAV_CMD_PREFLIGHT_CALIBRATION command"<<endl;

	return false;
}

bool APM_Interface::do_arm_disarm(int flag){

	if (command_long_send(MAV_CMD_COMPONENT_ARM_DISARM,flag,-1,-1,-1,-1,-1,-1))
		return true;
	else 
		cout<<"[APM Interface] - do_arm_disarm experimented a problem on MAV_CMD_COMPONENT_ARM_DISARM command"<<endl;

				
	return false;

}

bool APM_Interface::do_takepict() {

		/*Camera.grab();
    //allocate memory
    unsigned char *data=new unsigned char[  Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB )];
    //extract the image in rgb format
    Camera.retrieve ( data,raspicam::RASPICAM_FORMAT_RGB );//get camera image
    //save
    std::ofstream outFile ( "raspicam_image.ppm",std::ios::binary ); //INTRODUCIRLA DENTRO DEL PICTURES
    outFile<<"P6\n"<<Camera.getWidth() <<" "<<Camera.getHeight() <<" 255\n";
    outFile.write ( ( char* ) data, Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB ) );
    
    //free resrources    
    delete data;*/

		return true;
}

bool APM_Interface::do_takeoff(TokenTC* tk){

	/*TakingOff Procedure*
	1. Get altitude from TokenTC
	2. Clear previous missions from APM and items_map
	3. Write home position, take off mission item and Loiter Unlimited mission item on a list and write in the APM
	4. Start mission (with RC Override) 
	*/

	/*1.*/		
	float alt = 0;
	list<Attribute>::iterator ats = tk->attributes.begin();

	for(; ats !=tk->attributes.end(); ++ats){
		if (ats->getName().find("z")!=std::string::npos){
			alt = ats->getValue();
		}
	}

	if (alt<=0){
		cout<<"[APM Interface] - do_takeoff failed because the altitude is less than 0"<<endl;
		return false;
	}
		
	/*2.*/
	mission_item_seq = 0;
	
	if (do_clear_mission()){
		
		/*3.*/
		mavlink_message_t msg_home;
		//std::map<int,mavlink_message_t> items_map;

		msg_home = build_item(1,-1,-1,-1,-1,-1,-1,MAV_CMD_DO_SET_HOME, mission_item_seq);
		//items_map.insert(std::pair<int,mavlink_message_t>(0, msg_home));
		global_mission_map.insert(std::pair<int,mavlink_message_t>(mission_item_seq, msg_home));
		mission_item_seq++;

		mavlink_message_t msg_takeoff;
		msg_takeoff = build_item(1,-1,-1,-1, 0, 0, alt, MAV_CMD_NAV_TAKEOFF, mission_item_seq);
		//items_map.insert(std::pair<int,mavlink_message_t>(1, msg_takeoff));
		global_mission_map.insert(std::pair<int,mavlink_message_t>(mission_item_seq, msg_takeoff));
		waypoint_seq = mission_item_seq;		
		mission_item_seq++;

		/*Loiter cause to the UAV to be waiting in the current location unlimited*/
		mavlink_message_t msg_loiter;
		msg_loiter = build_item(-1,-1,-1,-1, 0, 0, 0, MAV_CMD_NAV_LOITER_UNLIM, mission_item_seq);
		global_mission_map.insert(std::pair<int,mavlink_message_t>(mission_item_seq, msg_loiter));
		//items_map.insert(std::pair<int,mavlink_message_t>(2, msg_loiter));
		mission_item_seq++;

		if (write_items_map())
		/*4.*/
			return do_start_goal(waypoint_seq);
	}

	return false;
}

bool APM_Interface::do_flyingto(TokenTC* tk) {

/*Flyingto Procedure*
	1. Get altitude, longitud and latitud from TokenTC
	2. Build waypoint and loiter unlimited
	3. Write Set current mission as waypoint
	*/
	float alt = 0;
	float lat = 0;
	float lon = 0;

	/*1.*/
	list<Attribute>::iterator ats = tk->attributes.begin();

		for(; ats !=tk->attributes.end(); ++ats){
  		if (ats->getName().find("x")!=std::string::npos){
				lat = ats->getValue();
			}else if (ats->getName().find("y")!=std::string::npos){
				lon = ats->getValue();
			}else if (ats->getName().find("z")!=std::string::npos){
				alt = ats->getValue();
			}
		}
	/*2.*/

	//NAV_WAYPOINT
	mavlink_message_t msg_wp;
	msg_wp = build_item(1,-1,-1,-1,lat,lon,alt,MAV_CMD_NAV_WAYPOINT,mission_item_seq);
	global_mission_map.insert(std::pair<int,mavlink_message_t>(mission_item_seq, msg_wp));
	waypoint_seq = mission_item_seq;
	mission_item_seq++;
	//LOITER_UNLIM
	mavlink_message_t msg_loiter;
	msg_loiter = build_item(-1,-1,-1,-1, 0, 0, 0, MAV_CMD_NAV_LOITER_UNLIM, mission_item_seq);
	global_mission_map.insert(std::pair<int,mavlink_message_t>(mission_item_seq, msg_loiter));
	mission_item_seq++;

	if (write_items_map())
		/*3.*/
			return do_start_goal(waypoint_seq);

	return false;

}

bool APM_Interface::do_landingto(TokenTC* tk) {

/*LandingTo Procedure*
	1. Get longitud and latitud from TokenTC
	2. Build waypoint NAV_LAND alone!!!!
	3. Write Set current mission as waypoint
	*/

	float lat = 0;
	float lon = 0;

	/*1. If x or z = 0, the land will take place on its current location*/
	list<Attribute>::iterator ats = tk->attributes.begin();

		for(; ats !=tk->attributes.end(); ++ats){
  		if (ats->getName().find("x")!=std::string::npos){
				lat = ats->getValue();
			}else if (ats->getName().find("y")!=std::string::npos){
				lon = ats->getValue();
			}
		}

	/*2.*/

	//LAND	
	mavlink_message_t msg_wp;
	msg_wp = build_item(-1,-1,-1,-1,lat,lon,-1,MAV_CMD_NAV_LAND,mission_item_seq);
	global_mission_map.insert(std::pair<int,mavlink_message_t>(mission_item_seq, msg_wp));
	waypoint_seq = mission_item_seq;
	mission_item_seq++;
	if (write_items_map())
		/*3.*/
			return do_start_goal(waypoint_seq);

	return false;

}

bool APM_Interface::do_start_goal(int g) {

	if (1 == g){//Is the TakeOffGoal so its need an START_MISSION command_long and a RC_CHANNELS_OVERRIDE
		
		if (command_long_send(MAV_CMD_MISSION_START,-1,-1,-1,-1,-1,-1,-1)) {

			
			mavlink_message_t msg_rc;
			mavlink_rc_channels_override_t rc;
			rc.chan1_raw=0; //UINT16_MAX
			rc.chan2_raw=0;
			rc.chan3_raw=1100; //Throttle
			rc.chan5_raw=0;
			rc.chan6_raw=0;
			rc.chan7_raw=0;
			rc.chan8_raw=0;
			rc.target_system = vehicle_id;
			rc.target_component = autopilot_id;
			mavlink_msg_rc_channels_override_encode(gcs_id, gcs_id, &msg_rc, &rc);

			return (1 < write_message(msg_rc));
		}else
			cout << "[APM Interface] - do_start_goal experimented a problem executing MAV_CMD_MISSION_START" << endl;
	}else {

			/*Set Current waypoint, jumping to the expecific mission item*/
			mavlink_message_t msg_sc;
			mavlink_mission_set_current_t sc;
			sc.seq = g;
			sc.target_system = vehicle_id;
			sc.target_component = autopilot_id;
			mavlink_msg_mission_set_current_encode(gcs_id, gcs_id, &msg_sc, &sc);
	
			if (1 < write_message(msg_sc))
			{

				double wait_time = 0; // in seconds
				clock_t t_ini;

				t_ini = clock();
				wait_time = (double)(clock()-t_ini) / CLOCKS_PER_SEC;

				//Wait to receive the waypoint_current which im waiting with cuenta atrás
				while (mavlink_data.mission_current.seq != g && wait_time < 5) //seconds
				{
					wait_time = (double)(clock()-t_ini) / CLOCKS_PER_SEC;
					
					pthread_mutex_lock(&lock_current);
					while (!waypoint_current)
					{
						pthread_cond_wait(&cv_current, &lock_current);
					}
					waypoint_current = false;
					pthread_mutex_unlock(&lock_current);
				}
				return mavlink_data.mission_current.seq == g;

			}
	}

	return false;
}

void APM_Interface::update_home_location() {
	
	home_location.latitud = mavlink_data.global_pos.lat;
	home_location.longitud = mavlink_data.global_pos.lon;
	home_location.altitud = mavlink_data.global_pos.alt;
}

bool APM_Interface::do_clear_mission() {

	mavlink_message_t msg_cl;
	mavlink_mission_clear_all_t clear;

	clear.target_system = vehicle_id;
	clear.target_component = autopilot_id;
	
	mavlink_msg_mission_clear_all_encode(gcs_id,gcs_id,&msg_cl, &clear);
	int len = write_message(msg_cl); 

	return (1 < len);
}


int APM_Interface::read_message(mavlink_message_t &message)
{
	uint8_t          cp;
	mavlink_status_t status;
	uint8_t          msgReceived = false;
	// this function locks the port during read
	int result = serial_port->_read_port(cp);
	if (result > 0)
	{
		// the parsing
		msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
		lastStatus = status;
	}

	// Couldn't read from port
	else
	{
		return -1;
	}
	return msgReceived;
}

//Write to APM via mavlink
int APM_Interface::write_message(const mavlink_message_t &message)
{
	char buf[256];

	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	// Write buffer to serial port, locks port while writing
	int bytesWritten = serial_port->_write_port(buf,len);

	return bytesWritten;
}

bool APM_Interface::set_flightmode(int mode)
{

		string mod = "";

		switch(mode) {
			case STABILIZE: 
						mod = "STABILIZE";break;
			case ACRO:
						mod = "ACRO";break;
			case ALT_HOLD:
						mod = "ALT_HOLD";break;
			case AUTO:
						mod = "AUTO";break;
			case GUIDED: 
					  mod = "GUIDED";break;
			case LOITER:
						mod = "LOITER";break;
			case RTL:
						mod = "RTL";break;
			case LAND:
						mod = "LAND";break;

		}

		mavlink_message_t msg;
		mavlink_set_mode_t nmode = { 0 };
		nmode.target_system      = vehicle_id;
		nmode.custom_mode        = mode;
		nmode.base_mode          = 1;
		mavlink_msg_set_mode_encode(gcs_id, gcs_id, &msg, &nmode);

		if (1 < write_message(msg))
		{
			cout << "[APM Interface] - Flight mode changed to "<<mod<<" successfully"<<endl;
			return true;
		}
		return false;

}

bool APM_Interface::command_long_send(int command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{

	  mavlink_message_t msg;
		mavlink_command_long_t cml = { 0 };
		cml.target_system          = vehicle_id;
		cml.target_component       = autopilot_id;
		cml.command                = command;
		cml.confirmation           = true;
		
		if (param1 != -1)
			cml.param1 = param1;
		if (param2 != -1)
			cml.param2 = param2;
		if (param3 != -1)
			cml.param3 = param3;
		if (param4 != -1)
			cml.param4 = param4;
		if (param5 != -1)
			cml.param5 = param5;
		if (param6 != -1)
			cml.param6 = param6;
		if (param7 != -1)
			cml.param7 = param7;
	
		mavlink_msg_command_long_encode(gcs_id, gcs_id, &msg, &cml);

		if (write_message(msg) > 1){
				//Waiting to receive a response of this command from APM
				pthread_mutex_lock(&lock);
				while (mavlink_data.command_ack.command != command)
				{
					pthread_cond_wait(&cv, &lock);
				}
				pthread_mutex_unlock(&lock);

				//Checking the result of this command
			int result = mavlink_data.command_ack.result;
			string response = "";
				switch (result){
						case MAV_RESULT_ACCEPTED:
							response = "Command ACCEPTED and EXECUTED";break;
						case MAV_RESULT_TEMPORARILY_REJECTED:
							response = "Command TEMPORARY REJECTED/DENIED"; break;
						case MAV_RESULT_DENIED:
							response = "Command PERMANENTLY DENIED"; break;
						case MAV_RESULT_UNSUPPORTED:
							response = "Command UNKNOWN/UNSUPPORTED"; break; 
						case MAV_RESULT_FAILED: 
							response = "Command EXECUTED, but FAILED"; break;
						default:
							response = "Null result"; break;
				}
			cout << "[APM Interface] - Command "<<command<< " result: "<<response<<endl;

			if (result==MAV_RESULT_ACCEPTED){
				sleep(1);
				return true;
			}
		}else
			cout << "[APM Interface] - Failing to send the command "<<command<<endl;

		return false;

}

mavlink_message_t APM_Interface::build_item(float param1, float param2, float param3, float param4, float x, float y, float z, int command, int seq)
{

	mavlink_message_t msg = { 0 };
	mavlink_mission_item_t item = { 0 };
	item.target_system          = vehicle_id;
	item.target_component       = autopilot_id;
	item.frame                  = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	item.current                = 1;
	item.autocontinue           = 0;

	if (param1 != -1)
		item.param1 = param1;
	if (param2 != -1)
		item.param2 = param2;
	if (param3 != -1)
		item.param3 = param3;
	if (param4 != -1)
		item.param4 = param4;

	if (x != -1)
		item.x = x;
	if (y != -1)
		item.y = y;
	if (z != -1)	
		item.z = z;

	item.seq = seq;
	item.command = command;

	mavlink_msg_mission_item_encode(gcs_id,gcs_id,&msg,&item);

	return msg;

}


bool APM_Interface::write_items_map()
{
	int cnt = global_mission_map.size()-1;
	/*Pick up the first element of the map*/
	
	int last_item_seq = 0;//mission_item_seq-items_map.size();

	mavlink_message_t msg_c;
	mavlink_mission_count_t count;
	count.count = global_mission_map.size();
	count.target_system = vehicle_id;
	count.target_component = autopilot_id;
	
	mavlink_msg_mission_count_encode(gcs_id,gcs_id,&msg_c,&count);
	
	/*Waypoint protocol start*/
	if (1 < write_message(msg_c)){
		while (cnt>0){

			pthread_mutex_lock(&lock);
			while (!item_request)
			{
				pthread_cond_wait(&cv, &lock);
			}
			item_request = false;
			pthread_mutex_unlock(&lock);
			map<int, mavlink_message_t>::iterator it = global_mission_map.begin();
			for( ; global_mission_map.end()!=it; ++it ) {
				if (it->first==mavlink_data.mission_request.seq){
					//cout << "ENVIANDO ITEM "<<it->first<<endl;
					cout << "[APM Interface] - Sending Mission Item: "<<it->first<<endl;
					write_message(it->second);
					break;
				}
			}
			//Updates item[n] when we received the item[n+1]
			if (mavlink_data.mission_request.seq==last_item_seq+1){
				last_item_seq=mavlink_data.mission_request.seq;
				cnt--;
			}
			/*Elapsed time between mission items*/
			sleep(SECONDS);
		}

		//cout << "ESPERANDO ACK" <<endl;
		pthread_mutex_lock(&lock);
		while (!mission_ack)
		{
			pthread_cond_wait(&cv, &lock);
		}
		mission_ack = false;
		pthread_mutex_unlock(&lock);

		int result = mavlink_data.mission_ack.type;

		string response = "";
		switch (result){
				case MAV_MISSION_ACCEPTED:
					response = "mission accepted OK";break;
				case MAV_MISSION_ERROR:
					response = "generic error / not accepting mission commands at all right now"; break;
				case MAV_MISSION_UNSUPPORTED_FRAME:
					response = "coordinate frame is not supported"; break;
				case MAV_MISSION_UNSUPPORTED:
					response = "command is not supported"; break; 
				case MAV_MISSION_NO_SPACE: 
					response = "mission item exceeds storage space"; break;
				case MAV_MISSION_INVALID:
					response = "one of the parameters has an invalid value";break;
				case MAV_MISSION_INVALID_PARAM1:
					response = "param1 has an invalid value"; break;
				case MAV_MISSION_INVALID_PARAM2:
					response = "param2 has an invalid value"; break;
				case MAV_MISSION_INVALID_PARAM3:
					response = "param3 has an invalid value"; break; 
				case MAV_MISSION_INVALID_PARAM4: 
					response = "param4 has an invalid value"; break;
				case MAV_MISSION_INVALID_PARAM5_X:
					response = "x/param5 has an invalid value";break;
				case MAV_MISSION_INVALID_PARAM6_Y:
					response = "y/param6 has an invalid value"; break;
				case MAV_MISSION_INVALID_PARAM7:
					response = "param7 has an invalid value"; break;
				case MAV_MISSION_INVALID_SEQUENCE:
					response = "received waypoint out of sequence"; break; 
				case MAV_MISSION_DENIED: 
					response = "not accepting any mission commands from this communication partner"; break;
				default:
					response = "Null"; break;
			}
		cout << "[APM Interface] - Mission writing result: "<<response<<endl;
		
		return (result==MAV_MISSION_ACCEPTED);
	}
	return false;
}

void APM_Interface::read_messages()
{
	bool received_all = false;  // receive only one message
	int attempt = 0;
	Time_Stamps this_timestamps;
	char text;

	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		mavlink_message_t message;
		
		if(read_message(message))
		{
			mavlink_data.sysid  = message.sysid;
			mavlink_data.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{
				
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					mavlink_msg_heartbeat_decode(&message, &(mavlink_data.heartbeat));
					mavlink_data.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = mavlink_data.time_stamps.heartbeat;
					
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					pthread_mutex_lock(&lock_battery);
					//cout << "\n[APM Interface] - TM ("<<MAVLINK_MSG_ID_SYS_STATUS<<") : SYS_STATUS"<<endl;
					mavlink_msg_sys_status_decode(&message, &(mavlink_data.sys_status));
					mavlink_data.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = mavlink_data.time_stamps.sys_status;
					pthread_mutex_unlock(&lock_battery);	
					pthread_cond_signal(&cv_battery);		
					break;
				}
				
				case MAVLINK_MSG_ID_GPS_RAW_INT:
				{
					//cout << "\n[APM Interface] - TM ("<<MAVLINK_MSG_ID_GPS_RAW_INT<<") : GPS_RAW_INT"<<endl;
					
					pthread_mutex_lock(&lock_gps);
					mavlink_msg_gps_raw_int_decode(&message, &(mavlink_data.gps_raw_int));
					mavlink_data.time_stamps.gps_raw_int = get_time_usec();
					this_timestamps.gps_raw_int = mavlink_data.time_stamps.gps_raw_int;
					pthread_mutex_unlock(&lock_gps);	
					pthread_cond_signal(&cv_gps);		
					break;
				}
				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					mavlink_msg_global_position_int_decode(&message, &(mavlink_data.global_pos));
					break;
				}
				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					break;
				}
				case MAVLINK_MSG_ID_HOME_POSITION:
				{
					break;
				}
				case MAVLINK_MSG_ID_COMMAND_ACK:
				{
					pthread_mutex_lock(&lock);
					mavlink_msg_command_ack_decode(&message, &(mavlink_data.command_ack));
					pthread_mutex_unlock(&lock);	
					pthread_cond_signal(&cv);			
		
					break;
				}
				case MAVLINK_MSG_ID_MISSION_REQUEST:
				{

					cout << "[APM Interface] - TM ("<<MAVLINK_MSG_ID_MISSION_REQUEST<<") : MISSION_REQUEST - "<<mavlink_msg_mission_request_get_seq(&message)<<endl;
					pthread_mutex_lock(&lock);
					mavlink_msg_mission_request_decode(&message, &(mavlink_data.mission_request));
					item_request = true;
					pthread_mutex_unlock(&lock);	
					pthread_cond_signal(&cv);			
		
					break;
				}
				case MAVLINK_MSG_ID_MISSION_ACK:
				{
					
					pthread_mutex_lock(&lock);
					mavlink_msg_mission_ack_decode(&message, &(mavlink_data.mission_ack));
					mission_ack = true;
					pthread_mutex_unlock(&lock);	
					pthread_cond_signal(&cv);

					break;
				}		
				case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
				{
					cout << "[APM Interface] - TM ("<<MAVLINK_MSG_ID_MISSION_ITEM_REACHED<<") : MISSION_ITEM_REACHED :: "<<mavlink_msg_mission_item_reached_get_seq(&message)<<endl;

					mavlink_msg_mission_item_reached_decode(&message, &(mavlink_data.mission_item_reached));


					break;
				}
				case MAVLINK_MSG_ID_MISSION_CURRENT:
				{
					//cout << "[APM Interface] - TM ("<<MAVLINK_MSG_ID_MISSION_CURRENT<<") : NEW_MISSION_CURRENT :: "<<mavlink_msg_mission_current_get_seq(&message)<<endl;
					pthread_mutex_lock(&lock_current);
					mavlink_msg_mission_current_decode(&message, &(mavlink_data.mission_current));
					waypoint_current = true;
					pthread_mutex_unlock(&lock_current);	
					pthread_cond_signal(&cv_current);		
					break;
				}
		    case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK:
				{
					break;
				}	
				case MAVLINK_MSG_ID_STATUSTEXT:
				{
					break;
				}
				default:
				{
					//printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}
			} 
		} 

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat   &&
				this_timestamps.sys_status  &&
				this_timestamps.gps_raw_int               
				;

	} // end while

	return;
}

vector<float> APM_Interface::getApmTelemetry(){

	vector<float> ret;
	ret.assign(3,0);

	ret.at(0) = mavlink_data.sys_status.voltage_battery/1000.0f;
	ret.at(1) = mavlink_data.gps_raw_int.fix_type;
	ret.at(2) = mavlink_data.sys_status.load/10.0f;

	return ret;


}



