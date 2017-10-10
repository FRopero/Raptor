# Welcome to Raptor's Architecture
Raptor's Architecture is an autonomous controller executing high-level layers and a functional layer executed on an onboard computer in a UAV.

The files commited in this repository are referred to the functional layer of the UAV. The goal of this layer is to communicate the high-level layers of the autonomous controller with the UAV. Then, it receives actions from the high-level layers, it translates to the low-level layer's language and then, send them to execution.

![Raptor](/extra/RaptorArchitecture.PNG)

## How to get it
### Build it
- Before to build it, you have to modify the variable ```PATH_EXE``` in the makefile. This is the location where the executable is going to be allocated. Then, you must follow the next steps:
```sh
$ cd Raptor-master
$ make
```

## Video
- https://www.youtube.com/watch?v=yryqd7gPUno


## Contact
Editor: Fernando Ropero - fernando.ropero@uah.es

## License

This project belongs to Fernando Ropero, PhD Student at the Computer Engineering Department, Universidad de Alcalá. 
