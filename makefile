OBJS = Drone_Server.o Drone_Interface.o  APM_Interface.o ServerSelect.o Serial_Port.o TokenTC.o Attribute.o crc.o TelemetryReporter.o
PATH_OBJS = objects
#PATH_EXE = /home/pi/Escritorio/DRONE/bin
PATH_EXE = /opt/goac/GOAC/TREX/bin
PATH_SERVER = /opt/goac/GOAC/TREX/extra/ogr/Server
CFLAGS = -c 
FLAGS = -lpthread -o 

all: Drone_Interface

Drone_Interface: $(OBJS)
	cd objects ; g++ $(OBJS) $(FLAGS) $(PATH_EXE)/Drone_Interface ; cd .. 

Drone_Server.o: Drone_Server.cpp Drone_Interface.h
	g++ $(CFLAGS) Drone_Server.cpp -o $(PATH_OBJS)/Drone_Server.o

Drone_Interface.o: Drone_Interface.h Drone_Interface.cpp APM_Interface.h Serial_Port.h ServerSelect.h TokenTC.h
	g++ $(CFLAGS) Drone_Interface.cpp -o $(PATH_OBJS)/Drone_Interface.o

APM_Interface.o: APM_Interface.h APM_Interface.cpp Drone_Interface.h TokenTC.h Serial_Port.h
	g++ $(CFLAGS)  APM_Interface.cpp -o $(PATH_OBJS)/APM_Interface.o

ServerSelect.o: ServerSelect.h ServerSelect.cpp 
	g++ $(CFLAGS) ServerSelect.cpp -o $(PATH_OBJS)/ServerSelect.o

Serial_Port.o: Serial_Port.h Serial_Port.cpp
	g++ $(CFLAGS) Serial_Port.cpp -o $(PATH_OBJS)/Serial_Port.o

TokenTC.o: TokenTC.h TokenTC.cpp Attribute.h
	g++ $(CFLAGS) TokenTC.cpp -o $(PATH_OBJS)/TokenTC.o

Attribute.o: Attribute.h Attribute.cpp
	g++ $(CFLAGS) Attribute.cpp -o $(PATH_OBJS)/Attribute.o

crc.o: crc.hh crc.cc
	g++ $(CFLAGS) crc.cc -o $(PATH_OBJS)/crc.o

TelemetryReporter.o: TelemetryReporter.h TelemetryReporter.cpp Serial_Port.h ServerSelect.h
	g++ $(CFLAGS) TelemetryReporter.cpp -o $(PATH_OBJS)/TelemetryReporter.o

clean:
	rm -rf $(PATH_OBJS) 
	mkdir $(PATH_OBJS)

