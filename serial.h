#ifndef SERIAL
#define SERIAL

#include <stdio.h>     // printf
#include <fcntl.h>     // open
#include <string.h>    // bzero
#include <stdlib.h>    // exit
#include <sys/times.h> // times
#include <sys/types.h> // pid_t
#include <termios.h>   //termios, tcgetattr(), tcsetattr()
#include <unistd.h>
#include <sys/ioctl.h> // ioctl

enum DATATYPE { IMAGE, INFO};

int openPort(const char *dev_name);

int configurePort(int fd);

bool sendXYZ(int fd, double *xyz);

bool sendData(int fd, char *data, int size, DATATYPE data_tpye);

bool sendXYZ_NEW(int fd, double *xyz);

#endif