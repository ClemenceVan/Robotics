#include "../pch.h"

#include "Arena.hpp"
#include "../Lidar/Lidar.hpp"
#include "../Motors/Motors.hpp"
#include "../Camera/Camera.hpp"
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
        Camera *camera;
        /* Threads */
        std::thread loopTh;
        std::thread motorsTh;
        std::thread displayTh;
        std::thread batteryTh;
        bool _running = false;

        /* Dev */
        double rho = 0;
        double gamma = 0;
        double delta = 0;
        double c1 = 0;
        double c2 = 0;


            std::ofstream kalman_stream;
    public:
        Robot(Arena *arena, bool display = false);

        ~Robot();

        void loop();

        void start();

        void run();

        void kalman();

        void reverse();

        bool velocity_profile(double end_x, double end_y, double end_a, double distance = NAN);

        void discover();

        void setFlag(std::string flag, bool value) {
            if (flag == "window") {
                camera->setWindowFlag(value);
            }
        }

        void debug(double a1, double a2, double a3, double c1, double c2) {
            this->rho = a1;
            this->gamma = a2;
            this->delta = a3;
            
            this->c1 = c1;
            this->c2 = c2;
        }

        void rotate(double desired_angle,double angle_velocity);
};