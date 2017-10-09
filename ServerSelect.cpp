/* 
 * File:   ServerSelect.cpp
 * Author: Pablo Muñoz
 * Date: June 7, 2014
 *
 * Implementation of a non-blocking server to serve multiple clients - DEPRECATED
 *
 * Modified by Fernando Ropero
 * Blocking server 
 */

#include "ServerSelect.h"


ServerSelect::ServerSelect() {
    numClients = 0;
    lastClient = 0;
    max = 0;
}

ServerSelect::~ServerSelect() {
    cleanKeys();
    for(int i=0; i<numClients; i++)
        close(clients[i]);
    close(socketServer);
}

bool ServerSelect::open(int port)
{
    // Check port number
    if(port < 0 || port > 65535)
        return -1;

	// Open the socket
	socketServer = socket(AF_INET, SOCK_STREAM, 0);
	if(socketServer < 0)
	 	return -1;

	// Prepare bind
	struct sockaddr_in dir;
	dir.sin_family = AF_INET;
	dir.sin_port = ntohs(port);
	dir.sin_addr.s_addr = INADDR_ANY;
	if(bind(socketServer, (struct sockaddr *)&dir, sizeof(dir)) == -1)
	{
		//std::cout<< "Socket cerrado"<<std::endl;
		close(socketServer);
		return -1;
	}

	// Accept clients
	if(listen(socketServer, 1) == -1)
	{
		close(socketServer);
		return -1;
	}
    return true;
}

bool ServerSelect::isOpen()
{
    return socketServer >= 0;
}

int ServerSelect::receive(char* data)
{
    // Remove disconnected clients
    cleanKeys();
    // Initialize descriptors
    FD_ZERO(&readDescriptors);
    FD_SET(socketServer, &readDescriptors);
    // Set connected clients
    for(int i=0; i<numClients; i++)
        FD_SET(clients[i], &readDescriptors);

    getMaximum();
    if(max < socketServer)
        max = socketServer;

    // Read socket data
    select(max+1, &readDescriptors, NULL, NULL, NULL); // 3er NULL = &tmout;; NULL = blocking server

    // Check if a client has send data
    int readed;
    for(int i=0; i<numClients; i++)
        if(FD_ISSET(clients[i], &readDescriptors))
        {
            //readed = receiveSocket(clients[i], data);

						readed = read(clients[i],data,800);

            if(readed < 0)
                clients[i] = -1;
            else
            {
                lastClient = i;
                return readed;
            }
        }
    // Accept new clients
    if(FD_ISSET(socketServer, &readDescriptors)){
        newClient();
		}
    return 1;
}

int ServerSelect::sendData(char* data)
{
  if(lastClient >= 0 && lastClient < numClients)
      if(clients[lastClient] != -1){
					int s = send(clients[lastClient], data, strlen(data), 0);
					return s;
			}
  return -1;
}

// DUE TO RUN CONDITIONS IF A CLIENT DISCONNECTS, THIS FUNCTION FAILS
// the client list is not updated properly
int ServerSelect::sendAll(char* data)
{
  int total = 0;
  for (int i=0; i<numClients; i++)
      if(clients[i] != -1)
          total += send(clients[i], data, strlen(data), 0);

  return total;
}

bool ServerSelect::newClient()
{
    // Check maximum number of client allowed
    if(numClients >= MAX_CLIENTS)
    	return false;
    
    // Accept client
	socklen_t lenClient;
	struct sockaddr CLIENT;
	lenClient = sizeof(CLIENT);
	clients[numClients] = accept(socketServer, &CLIENT, &lenClient);
    if(clients[numClients] < 0)
        return -1;

    // If client is connected, send its client number
    char clientid[M_LENGTH];
    sprintf(clientid, "%d", numClients);
    send(clients[numClients], clientid,strlen(clientid),0);
    numClients++;
		//std::cout << "[Server] - New client connected to the server. ID: "<<clientid<<std::endl;
    return true;
}

void ServerSelect::getMaximum()
{
    if(numClients < 1)
	    max = 0;
    else
    {
    	max = clients[0];
	    for(int i=0; i<numClients; i++)
            if(clients[i] > max)
                max = clients[i];
    }
}

void ServerSelect::cleanKeys()
{
    if(numClients > 0)
    {
        int j = 0;
        for(int i=0; i<numClients; i++)
            if(clients[i] != -1)
            {
                clients[j] = clients[i];
		        j++;
            }
        numClients = j;
    }
}

int ServerSelect::getSocket()
{
    return socketServer;
}
