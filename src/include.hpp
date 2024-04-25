
#include "../Eigen/Dense"

int screen(Eigen::MatrixXd positions, Eigen::MatrixXd new_positions);

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

#ifndef _SSIZE_T_DEFINED
    typedef intptr_t ssize_t;
    #define _SSIZE_T_DEFINED
#endif
// Define a function or macro named "close" to call closesocket
#ifdef _WIN32
    #define close closesocket
#endif