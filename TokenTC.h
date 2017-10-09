#ifndef TokenTC_H
#define TokenTC_H

#include "Attribute.h"
#include <list>
#include <sstream>
using namespace std;

class TokenTC
{
	public:
		/*Constructor*/
		TokenTC();
		TokenTC(int id_action);
		/*Destructor*/
		~TokenTC();

		/*Token Type -> CMD or REPLY or QUERY*/
		void setType(std::string type);
		string getType();
		
		/*Timeline of the token*/
		void setTM(std::string tm);
		string getTM();

		/*Predicate of the token*/
		void setPredicate(std::string pred);
		std::string getPredicate();
	
		void setTick(int clk);
		int getTick();
		
		/*Attributes of the token*/
		list<Attribute> attributes;

		string generateMsg();

		bool reached; //If the action was accomplish or not
		bool executed; //If the action was sended to the APM
		int id; //action id

	private:
		string token_type; //CMD or REPLY or QUERY
		int tick; //Tick when it was send to the drone
		string timeline;
		string predicate;

		
		
		

};

#endif
