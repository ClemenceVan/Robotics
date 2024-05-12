#include "../include.hpp"
#include "../../libraries/spi_com.h"
#include "../../libraries/current/ina219.h"
#include "../Robot/Arena.hpp"
#ifndef _WIN32
    #include <wiringPi.h>
    #include <wiringPiSPI.h>
#else
#endif

// void Send_Read_Motor_Data(&MotorData) {}
class Motors {
    private:
        Arena arena;
        double posX;
        double posY;
        double posA;
        double prevPosX;
        double prevPosY;
        double prevPosA;
        double Dr = 0;
        double Dl = 0;
        double prev_enc_l = 0;
        double prev_enc_r = 0;
        Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd Cxya_old = Eigen::MatrixXd::Zero(3, 3);

        MotorDataType MotorData;
        double offset_m1 = 0;
        double offset_m2 = 0;
        
        double wheel_base = 7; // cm
        double wheel_diameter = 3; // cm
        double PULSES_PER_REVOLUTION = 1024; // ticks
        double circumference_wheel = wheel_diameter * M_PI;
        double MM_PER_PULSE = circumference_wheel / PULSES_PER_REVOLUTION;
        double gearbox_ratio = 10;
        std::mutex positionMutex;
    
        std::thread readWriteTh;
    public:
        Motors(Arena arena);
        ~Motors() {}

        std::pair<double, double> refreshEncoders();

        void odometry();

        void updatePosition(double x, double y, double a, Eigen::MatrixXd cov);

        double getPosX();

        double getPosY();

        double getPosA();

        Eigen::MatrixXd getCovariance();

        void setSpeed(int left, int right);

        void velocity_profile(double end_x, double end_y, double end_a);
};