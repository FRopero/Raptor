#include <iostream>
#include "Attribute.h"

Attribute::Attribute(std::string name, float value){
	myName = name;
	myValue = value;
}

Attribute::~Attribute(){}

std::string Attribute::getName(){
	return myName;
}

int Attribute::getValue(){
	return myValue;
}
