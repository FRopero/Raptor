#ifndef APM_INTERFACE_H
#define APM_INTERFACE_H


#include <iostream>
#include <vector>
#include <sys/time.h>
#include "mavlink/v1.0/common/mavlink.h"
#include "Drone_Interface.h"
#include "TokenTC.h"
#include "Serial_Port.h"
#include "Attribute.h"
#include <fstream>
#include <map>
#include <time.h>
#include <vector>
#include "raspicam-0.1.4/src/raspicam.h"

//Connection
#define CONNECT_ATTEMPTS 10
#define REPEAT_READING 10 


//Flight Modes
#define STABILIZE 0
#define ACRO      1
#define ALT_HOLD  2
#define AUTO      3
#define GUIDED    4
#define LOITER    5
#define RTL       6
#define LAND      9

//Attempts to execute the goals before to execute the Land failsafe
#define AT_TAKEOFF 3
#define AT_LAND 3
#define AT_FLYINGTO 3
#define AT_TAKEPICT 3

//Offsets in mission items reached (in meters)
#define ALTITUD_OFFSET 0.25
#define LATITUD_OFFSET 0.25
#define LONGITUD_OFFSET 0.25

//Elapsed time limits in mission items (in seconds)
#define ELAPSED_TIME_TAKEOFF 50
#define ELAPSED_TIME_FLYINGTO 50
#define ELAPSED_TIME_LANDING 50

//Elapsed time between mission items
#define SECONDS 0.75
#define INIT_SEC 0.75

//Battery bounds to enable the flight
#define START_FLIGHT_BATT 10.0
#define STOP_FLIGHT_BATT 10.0

//GPS Fix time (in seconds)
#define GPS_FIXING_TIME 1 


/*Thread which implement the control algorithm of the APM Interface*/
void* dispatch_thread(void *args);
/*Thread which is reading the telemetry that comes from the APM Autopilot*/
void* read_thread(void *args);

/*Returns the current tick of the system*/
uint64_t get_time_usec();

/*Structure to store the arrival time of the MAVLink messages*/
struct Time_Stamps
{
	Time_Stamps()
	{
		reset_timestamps();
	}

	uint64_t heartbeat;
	uint64_t sys_status;
	uint64_t gps_raw_int;

	void reset_timestamps()
	{
		heartbeat = 0;
		sys_status = 0;
		gps_raw_int = 0;
	}

};

struct Location {

	uint32_t latitud; /*< Latitude, expressed as degrees * 1E7*/
	uint32_t longitud;/*< Longitud, expressed as degrees * 1E7*/
	uint32_t altitud; /*< Altitud, expressed as millimeters over the sea level*/
};


/*Structura to store all the information of the MAVLink messages. This structure is updating 
the information as the MAVLink messages come from the APM. Therefore, it only stores the last message information.*/
struct Mavlink_Data {

	int sysid;
	int compid;

	// Heartbeat
	mavlink_heartbeat_t heartbeat;

	// System Status
	mavlink_sys_status_t sys_status;

	// Battery Status
	mavlink_battery_status_t battery_status;

	// Global Position
	mavlink_global_position_int_t global_position_int;

	//GPS RAW
	mavlink_gps_raw_int_t gps_raw_int;

	//GLOBAL pos
	mavlink_global_position_int_t global_pos;

	//Mission Request
	mavlink_mission_request_t mission_request;

	//Mission Item
	mavlink_mission_item_t mission_item;

	//Mission Ack
	mavlink_mission_ack_t mission_ack;

	//Mission Current
	mavlink_mission_current_t mission_current;

	//Mission Item Reached
	mavlink_mission_item_reached_t mission_item_reached;

	//Command Ack
	mavlink_command_ack_t command_ack;

	//Time Stamps of the messages
	Time_Stamps time_stamps;

	void reset_timestamps()
	{
		time_stamps.reset_timestamps();
	}


};

class APM_Interface
{

public:
	/*Empty Constructor of the APM Interface*/
	APM_Interface();
	/*Constructor of the APM Interface*/
	APM_Interface(Drone_Interface *ogr,Serial_Port *serial_port_);
	/*Destructor*/
	~APM_Interface(); 
	
	/*Store the status of the last MAVLink message received*/
	mavlink_status_t lastStatus;

	/*ID's*/
  int vehicle_id;
	int autopilot_id;
	int gcs_id;

	/*Main store*/	
	Mavlink_Data mavlink_data;

	Location home_location;

	/*Init the inferface. Launch reader and dispatcher thread*/
	int start();
	/*Stop the interface. Wait to finish reader and dispatcher thread*/
	int stop();

	/*Control algorithm executed by thread dispatcher. Check actions to send them to execution 
    via MAVLink or set the flag Reached=True because it was completed.*/
	void dispatcher();

		/*Thread reader that read all the MAVLink messages with a certain frecuency*/
	void reader();
	void read_messages();
	
	void update_home_location();

	/*Send to execution the TokenTC*/
	bool token_execution_handler(TokenTC* tk);
	/*Wait the reached response*/
	bool token_response_handler(TokenTC* tk);

	/*Low level routines to check the reached*/
	bool wait_reached_takeoff();
	bool wait_reached_flyto();
	bool wait_reached_land();
	/*Low level execution actions with MAVLink*/
	bool do_takeoff(TokenTC* tk);
	int attempts_takeoff;
	bool do_flyingto(TokenTC* tk);
	int attempts_flyingto;
	bool do_landingto(TokenTC* tk);
	int attempts_landingto;
	bool do_precheck();
	bool do_arm_disarm(int flag); //arm = 1 ; disarm = 0
	bool do_takepict();

	/*Low level general actions with MAVLink*/
	bool do_start_goal(int first_i);
	bool do_clear_mission();


	/*Set up the main features of the APM autopilot*/
	int setup_APM();
	bool verifying_critical_systems();

	/*Represent the secuence number of the items mission*/
	uint16_t mission_item_seq;
	uint16_t waypoint_seq;
	bool mission_ack;

	bool failsafe;

	uint16_t list_seq;

	/*Low level method to read a mavlink_message from the serial port*/
	int read_message(mavlink_message_t &message);
	
	/*Low level method to write a mavlink_message by the serial port*/
	int write_message(const mavlink_message_t &message);

	bool set_flightmode(int mode);
	bool command_long_send(int command, float param1, float param2, float param3, float param4, float param5, float param6, float param7);
	bool write_items_map();
	mavlink_message_t build_item(float param1, float param2, float param3, float param4, float x, float y, float z, int command, int seq);
	
	pthread_mutex_t lock_action = PTHREAD_MUTEX_INITIALIZER;
  pthread_cond_t cv_action = PTHREAD_COND_INITIALIZER;
	
	vector<float> getApmTelemetry();
	

private:

	/*Communication with the drone*/
	Serial_Port *serial_port;
	/*Communication with ogr-trex*/
	Drone_Interface *Drone_interface;

	/*Dispatcher thread which implement and execute the control algorithm of the APM interface*/
	pthread_t dispatch_tid;
	/*Reader thread whose one goal is read all the mavlink messages received by the serial port*/
	pthread_t read_tid;

	/*Flag to allow us finishing the interface and their threads*/
	bool time_to_exit;

	bool item_request;
	bool waypoint_current;
	
	int item_reached_id;

	std::map<int,mavlink_message_t> global_mission_map;

	pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
  pthread_cond_t cv = PTHREAD_COND_INITIALIZER;
	pthread_mutex_t lock_battery = PTHREAD_MUTEX_INITIALIZER;
  pthread_cond_t cv_battery = PTHREAD_COND_INITIALIZER;
	pthread_mutex_t lock_gps = PTHREAD_MUTEX_INITIALIZER;
  pthread_cond_t cv_gps = PTHREAD_COND_INITIALIZER;
	pthread_mutex_t lock_current = PTHREAD_MUTEX_INITIALIZER;
  pthread_cond_t cv_current = PTHREAD_COND_INITIALIZER;
	//raspicam::RaspiCam Camera; //Camera object

};

#endif
