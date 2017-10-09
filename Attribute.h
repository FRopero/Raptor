#ifndef Attribute_H
#define Attribute_H

using namespace std;

class Attribute
{
	public:
		/*Constructor*/
		Attribute(std::string name, float value);
		/*Destructor*/
		~Attribute();
		
		std::string getName();
		void setName(string name);
		int getValue();
		void setValue();

	private:
		std::string myName;
		int myValue;

};

#endif
