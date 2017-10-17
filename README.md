# Welcome to Raptor's Architecture
Raptor's Architecture is an autonomous controller executing high-level layers and a functional layer executed on an onboard computer in a UAV.

The files commited in this repository are referred to the functional layer of the UAV. The goal of this layer is to communicate the high-level layers of the autonomous controller with the UAV. Then, it receives actions from the high-level layers, it translates to the low-level layer's language and then, send them to execution.

![Raptor](/extra/RaptorArchitecture.PNG)

## How to get it in Linux Systems
### Build it
- Before to build it, you have to check the next points:

* Install the mavlink protocol in the project
```sh
$ mkdir raptor0.1
$ mv mavlink raptor0.1\ 
```

* To modify the variable ```PATH_EXE``` in the makefile. This is the location where the executable is going to be allocated. 

- Then, you must follow the next steps:
```sh
$ cd raptor
$ make
```

### Run it
```sh
$ .\Drone_Interface [optional] -S [mandatory] [Option 1] -p PORT_SERVER [Option 2] -su SERVER_UART -sb BAUD_SERVER [mandatory] -u UART_APM -b BAUD_APM
````
-- [Optional]
- -S: enable or disable the Simulation Mode. 

-- [Mandatory]
- -p: from [Option 1] to indicate the port on which the server will listen. It is used when the server is executed in a local pc.
- -su: from [Option 2] to indicate the UART on which the server will connect with OGR. It is used for the communication between the functional layer and the deliberative layer.
- -sb: from [Option 2] to indicate the baudrate of the uart.
- -u: the uart port on which the Drone Server will connect with the APM Interface.
- -b: the baudrate of the UART.

- [Option 1]: This option will connect through TCP/IP
- [Option 2]: This option will connect through Serial
    
## Video
- https://www.youtube.com/watch?v=yryqd7gPUno


## Contact
Editor: Fernando Ropero - fernando.ropero@uah.es

## License

This project belongs to Fernando Ropero, PhD Student at the Computer Engineering Department, Universidad de Alcalá. 
