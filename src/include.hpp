#include <stdio.h>                 // for getchar
#include <cmath>                   // for cos
#include <ftxui/dom/elements.hpp>  // for Fit, canvas, operator|, border, Element
#include <ftxui/screen/screen.hpp>  // for Pixel, Screen
#include <vector>                   // for vector, allocator
#include <iostream>
#include <fstream>
#include <string>
#include <sys/types.h>
#include <thread>
#include <mutex>

#ifdef _WIN32
    #include <winsock2.h>
    #include <windows.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <unistd.h>
#endif
#include "Eigen/Dense"


#define PORT 9888

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#ifndef _SSIZE_T_DEFINED
    typedef intptr_t ssize_t;
    #define _SSIZE_T_DEFINED
#endif

#ifdef _WIN32
    #define close closesocket
    #define read(x, y, z) recv(x, y, z, 0)
    #define sleep(x) Sleep(1000 * x) // x in seconds
#endif
#ifndef _INIT_ARENA_
    #define _INIT_ARENA_
#endif

int screen(Eigen::MatrixXd positions, Eigen::MatrixXd new_positions);