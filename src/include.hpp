#include <stdio.h>                 // for getchar
#include <cmath>                   // for cos
#include <ftxui/dom/elements.hpp>  // for Fit, canvas, operator|, border, Element
#include <ftxui/screen/screen.hpp>  // for Pixel, Screen
#include <vector>                   // for vector, allocator
#include <iostream>

#include "Eigen/Dense"


#define PORT 9888

#ifndef _SSIZE_T_DEFINED
    typedef intptr_t ssize_t;
    #define _SSIZE_T_DEFINED
#endif
// Define a function or macro named "close" to call closesocket
#ifdef _WIN32
    #define close closesocket
#endif
#ifndef _INIT_ARENA_
    #define _INIT_ARENA_
    int width = 27;
    int height = 57;

    double start_x = 13.5; // to be changed when start
    double start_y = 10.5; // to be changed when start
#endif