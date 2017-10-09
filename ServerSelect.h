/* 
 * File:   ServerSelect.h
 * Author: Pablo Muñoz
 * Date: June 7, 2014
 *
 * A non-blocking server to serve multiple clients
 */

#ifndef SERVERSELECTOGR_H
#define	SERVERSELECTOGR_H

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <netdb.h>
#include <errno.h>
#include <iostream>
#include <unistd.h>
#include <stdio.h>

/** Maximun number of client allowed. */
#define MAX_CLIENTS 10
/** Nanosec for read timeout. */
#define TIMEOUT_NANO 10

#define M_LENGTH 800

class ServerSelect {
public:
    /** Empty constructor. */
    ServerSelect();

    /** Disconnect all clientes and close the server. */
    ~ServerSelect();

    /** Open a localhost server using the provided data.
     @param port port number for the server
     @return True if the server is properly open, false otherwise.
     */
    bool open(int port);

    /** Read data for the next client.
     @param data buffer to store the read data
     @return size of the data readed, -1 for I/O error.
     */
    int receive(char* data);

    /** Write data to the socket of the last client who sends data.
     @param data buffer to send to the client
     @return size of sended data, -1 for I/O error.
     */
    int sendData(char* data);

    /** Write data to all connected clients.
     @param data buffer to send to the client
     @return size of sended data, -1 for I/O error.
     */
    int sendAll(char* data);

    /** Check if the server is open.
     @return True if the server is properly open, false otherwise.
     */
    bool isOpen();

    /** @return Return the socket descriptor of the server. */
    int getSocket();

private:
    /** Allows the connection of a new client.
     @return True if the connection is accepted, false otherwise.
     */
    bool newClient();
    /** Computes the high number of the connected clients.*/
    void getMaximum();
    /** Clean the clients key, removing the keys for disconnected clients. */
    void cleanKeys();

    /** Server socket descriptor. */
    int socketServer;
    /** Set of client socket descriptors. */
    int clients[MAX_CLIENTS];
    /** Number of connected clients. */
    int numClients;
    /** Last client that has send data. */
    int lastClient;
    /* Data for select(). */
    fd_set readDescriptors;
    /** Value for the higher client descriptor. */
    int max;
    /** Timeout value for non-blocking server. */


    struct timeval tmout;
};

#endif	/* SERVERSELECT_H */

