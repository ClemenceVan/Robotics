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
        // double posX = 39/2;
        // double posY = 54/2;
        // double posA = 0;//90*M_PI/180;
        // double prevPosX = 39/2;
        // double prevPosY = 54/2;
        // double prevPosA = 0;//90*M_PI/180;
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
        // #ifndef M_PI
        // #define M_PI 3.14159265358979323846
        // #endif
        Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd Cxya_old = Eigen::MatrixXd::Zero(3, 3);

        MotorDataType MotorData;
        #ifndef _WIN32
            INA219 Ina(SHUNT_OHMS, MAX_EXPECTED_AMPS);
        #endif
        double offset_m1 = 0;
        double offset_m2 = 0;

        
        double wheel_base = 7; // cm
        double wheel_diameter = 3; // cm
        double PULSES_PER_REVOLUTION = 1024; // ticks
        double circumference_wheel = wheel_diameter * M_PI;
        double MM_PER_PULSE = circumference_wheel / PULSES_PER_REVOLUTION;
        double gearbox_ratio = 10;
        std::mutex positionMutex;
    public:
    Motors(Arena arena): arena(arena) {
        posX = arena.getOrigin().first;
        posY = arena.getOrigin().second;
        posA = 0;
        prevPosX = posX;
        prevPosY = posY;
        prevPosA = posA;
        #ifndef _WIN32
            wiringPiSetup(); 
            wiringPiSPISetup(SPI_Channel, 1000000);
            Ina.configure(RANGE_16V, GAIN_8_320MV, ADC_12BIT, ADC_12BIT);
        #endif

        MotorDataType motor;
        Send_Read_Motor_Data(&MotorData);
        
        // printf("\u001b[35m\n");
        
        Send_Read_Motor_Data(&MotorData);
        offset_m1 = MotorData.Encoder_M1;
        offset_m2 = -MotorData.Encoder_M2;
    }
    ~Motors() {}

    std::pair<double, double> refreshEncoders() {
        Send_Read_Motor_Data(&MotorData);
        return std::make_pair(-(MotorData.Encoder_M1) - offset_m1, MotorData.Encoder_M2 - offset_m2);
    }

    void odometry() {
        std::pair<double, double> encoders = refreshEncoders();
        double current_enc_l = encoders.first;
        double current_enc_r = encoders.second;

        //----Kinematic model of a differential drive robot: ------
        double C = PULSES_PER_REVOLUTION * 6.6 * gearbox_ratio * 4 / circumference_wheel; // C = Nbr of pulses per mm = pulses per revolution * gearbox ratio * 4 (counts per puls) /circumference wheel
        double dt = 0.05;//50 / 1000; // sample time
        double vl = (current_enc_l - prev_enc_l) / (dt*C);
        double vr = (current_enc_r - prev_enc_r) / (dt*C);

        double v = (vr+vl)/2;
        double w = (vr-vl)/wheel_base;
        double L = v*dt;
        //----Movement in robot coordinate system:-----
        double dA = w*dt; // A = theta = angle
        double dX =  L * cos(dA/2);
        double dY = L * sin(dA/2);
        
        //-----covariance stuff: -------

        float SIGMAl = 0.5/20;// uncertainty for left encoder
        float SIGMAr = 0.5/20;

        float dDr = vr;//(current_enc_l - prev_enc_l)*MM_PER_PULSE;
        float dDl = vl;//(current_enc_r - prev_enc_l)*MM_PER_PULSE; // should just use vr and vl here maybe instead ?? 

        float dD = v; //(dDr + dDl)/2 ; 
        float dA2 = dA;//(dDr - dDl)/wheel_base ;  

        Eigen::MatrixXd Axya(3, 3);
        Axya << 1, 0, -dD * sin(prevPosA + dA2/2),
                0, 1, dD * cos(prevPosA + dA2/2),
                0, 0, 1;
                
        Eigen::MatrixXd Au(3, 2);
        Au << cos(prevPosA + dA2/2), -dD*0.5*sin(prevPosA+ dA2/2),
            sin(prevPosA + dA2/2), dD*0.5*cos(prevPosA+ dA2/2),
            0,1;  
            

        float CV = (pow(SIGMAr,2) - pow(SIGMAl,2))/(2*wheel_base);//*dt*C);
        
        Eigen::MatrixXd Cu(2, 2);
        Cu  << (pow(SIGMAr,2) + pow(SIGMAl,2))/4,CV,
                CV,(pow(SIGMAr,2)+ pow(SIGMAl,2))/(pow(wheel_base,2));

        // std::cout << "Axya = " << Axya << std::endl;
        // std::cout << "Cxya_old = " << Cxya_old << std::endl;
        // std::cout << "Au = " << Au << std::endl;
        // std::cout << "Cu = " << Cu << std::endl;

        covariance = Axya*Cxya_old*Axya.transpose() + Au*Cu*Au.transpose(); //Cxya_new in matlab code
        
        // std::cout << "covaraince odometry: " << covariance << std::endl;

        /* create position in global coordinate system */
        positionMutex.lock();
        posX = prevPosX + dX*cos(prevPosA) - dY*sin(prevPosA);
        posY = prevPosY + dY*cos(prevPosA) - dX*sin(prevPosA);
        posA = prevPosA + dA;

        
        std::cout << "Odometry Position: x = " << posX << ", y = " << posY << ", angle = " << posA << std::endl;
        /* update values */
        prevPosX = posX;
        prevPosY = posY;
        prevPosA = posA;
        positionMutex.unlock();
        prev_enc_l = current_enc_l;
        prev_enc_r = current_enc_r;
        Cxya_old = covariance;
        
    }

    void updatePosition(double x, double y, double a) {
        positionMutex.lock();
        prevPosX = x; // right coordinates to update ?
        prevPosY = y;
        prevPosA = a;
        posX = x;
        posY = y;
        posA = a;
        positionMutex.unlock();
    }

    double getPosX() {
        return posX;
    }

    double getPosY() {
        return posY;
    }

    double getPosA() {
        return posA;
    }

    Eigen::MatrixXd getCovariance() {
        return covariance;
    }
};