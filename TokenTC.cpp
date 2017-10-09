#include <iostream>
#include "TokenTC.h"

TokenTC::TokenTC(){
	token_type = "";
	tick = 0;
	predicate = "";
	timeline = "";
	id = -1;
	reached = false;
	executed = false;
}
TokenTC::TokenTC(int id_action){
	token_type = "";
	tick = 0;
	predicate = "";
	timeline = "";
	id = id_action;

	reached = false;
	executed = false;
}	

TokenTC::~TokenTC(){}

void TokenTC::setTM(std::string tm){
	timeline = tm;
}
std::string TokenTC::getTM(){
	return timeline;
}

void TokenTC::setType(std::string type){
	token_type = type;
}
std::string TokenTC::getType(){
	return token_type;
}

void TokenTC::setPredicate(std::string pred){
	predicate = pred;
}

std::string TokenTC::getPredicate(){
	return predicate;
}

void TokenTC::setTick(int clk){
	tick = clk;
}

int TokenTC::getTick(){
	return tick;
}

string TokenTC::generateMsg(){

	std::stringstream reply; 

	if (token_type=="QUERY"){
		reply << "QUERY|" << tick << "|" << timeline << "|" << predicate <<"|";
	}else{
		reply << "REPLY|" << tick << "|" << timeline << "|" << predicate <<"|";
		std::list<Attribute>::iterator t2 = attributes.begin();
		for(; attributes.end()!=t2; ++t2 )
		{
			reply << (*t2).getName() << "|" << (*t2).getValue() << "|";
		}
	}

	return reply.str();
}



