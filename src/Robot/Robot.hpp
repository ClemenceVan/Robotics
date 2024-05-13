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
        double posA = 90*M_PI/180; //was 0
        /* Battery */
        static const int SPI_Channel = 1;
        /* Modules */
        Arena *arena;
        Lidar *lidar;
        Motors *motors;
        Display *display;
        /* Threads */
        std::thread lidarTh;
        std::thread motorsTh;
        std::thread displayTh;
        std::thread batteryTh;
        bool _running = false;
    public:
        Robot(Arena *arena, bool display = false);

        ~Robot();

        void start();

        void run();

        void kalman();

        void debug_velocity(double rho, double gamma, double delta) {
            motors->debug_velocity(rho, gamma, delta);
        }
};