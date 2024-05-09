#include "../include.hpp"

#include "Arena.hpp"
#include "../Lidar/Lidar.hpp"
#include "../Motors/Motors.hpp"
#include "../Display.hpp"

#ifdef _WIN32 // if on windows, use file as data source to simulate lidar
    #define PATH "../testfile90.txt"
#else
    #define PATH ""
#endif

class Robot {
    private:
        /* Coordinates */
        double posX = 0;
        double posY = 0;
        double posA = 0;
        /* Modules */
        Arena *arena;
        Lidar *lidar;
        Motors *motors;
        Display *display;
        /* Threads */
        std::thread lidarTh;
        std::thread motorsTh;

    public:
        Robot(Arena *arena);

        ~Robot();

        void start();

        void run();

        void kalman();
};