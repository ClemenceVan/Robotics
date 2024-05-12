#include "../../libraries/spi_com.h"

#ifdef _WIN32 // simulate motor data on windows
	#ifndef _READ_MOTOR_DATA_
		#define _READ_MOTOR_DATA_
		void Send_Read_Motor_Data(MotorDataType *MotorData) {
			MotorData->Set_Speed_M1 = 0;
			MotorData->Set_Speed_M2 = 0;
			MotorData->Act_Speed_M1 = 0;
			MotorData->Act_Speed_M2 = 0;
			MotorData->Encoder_M1 = 0;
			MotorData->Encoder_M2 = 0;
		}
	#endif
#endif

#include "./Motors.hpp"

Motors::Motors(Arena arena): arena(arena) {
    posX = arena.getOrigin().first;
    posY = arena.getOrigin().second;
    posA = 0;
    prevPosX = posX;
    prevPosY = posY;
    prevPosA = posA;

    // MotorDataType motor;
    // Send_Read_Motor_Data(&MotorData);
    
    // printf("\u001b[35m\n");
    
    Send_Read_Motor_Data(&MotorData);
    offset_m1 = MotorData.Encoder_M1;
    offset_m2 = -MotorData.Encoder_M2;

    this->readWriteTh = std::thread([this] {
        while(true) {
            usleep(100000);
            Send_Read_Motor_Data(&MotorData);
        }
    });
    this->readWriteTh.detach();
}

std::pair<double, double> Motors::refreshEncoders() {
    Send_Read_Motor_Data(&MotorData);
    return std::make_pair(-(MotorData.Encoder_M1) - offset_m1, MotorData.Encoder_M2 - offset_m2);
}

void Motors::updatePosition(double x, double y, double a, Eigen::MatrixXd cov) {
    positionMutex.lock();
    prevPosX = x; // right coordinates to update ?
    prevPosY = y;
    prevPosA = a;
    posX = x;
    posY = y;
    posA = a;
    Cxya_old = cov;
    positionMutex.unlock();
}

double Motors::getPosX() {
    return posX;
}

double Motors::getPosY() {
    return posY;
}

double Motors::getPosA() {
    return posA;
}

Eigen::MatrixXd Motors::getCovariance() {
    return covariance;
}

void Motors::setSpeed(int left, int right) {
    MotorData.Set_Speed_M1 = left;
	MotorData.Set_Speed_M2 = -right;
    Send_Read_Motor_Data(&MotorData);
}

void Motors::velocity_profile(double end_x, double end_y, double end_a)
{
    positionMutex.lock();
    // double dx = end_x - kalman_x;
    // double dy = end_y - kalman_y;
    double dx = end_x - posX;
    double dy = end_y - posY;
    double kalman_a = posA;
    positionMutex.unlock();
    std::cout << "velocity profile dx = " << dx << ", dy = " << dy << std::endl;
    double epsilon = atan2(dy,dx); // be careful of dx close to zero ? 
    std::cout << "epsilon = " << epsilon << std::endl;
    double d = sqrt(pow(dy,2)+pow(dx,2)); // distance to drive when we have turned;
    std::cout << "d = " << d << std::endl;
    double gamma = epsilon - kalman_a; // the angle we should turn to in global coordinate system to aim at new position
    std::cout << "gamma = " << gamma << std::endl;
    double delta = end_a - gamma - kalman_a; // desired angle in world
    std::cout << "delta = " << delta << std::endl;

    std::cout << "delta" << delta << std::endl;
    double k_rho = 0.1;
    double k_gamma = 0.1; 
    double k_delta = 0.1; // can change this value
    double rho = sqrt(dx*dx + dy*dy); // ?????? dont know value of this
    std::cout << "rho = " << rho << std::endl;
    double v_new = k_rho*rho;
    std::cout << "v_new = " << v_new << std::endl;
    double w_new = k_gamma * gamma + k_delta * delta;
    std::cout << "w_new = " << w_new << std::endl;
    double v_right =( v_new + w_new * wheel_base / 2) * 1000;
    double v_left = (v_new - w_new*wheel_base / 2) * 1000;
    std::cout << "v_left = " << v_left << ", v_right = " << v_right << std::endl;
    setSpeed(v_left > 7000 ? 7000 : v_left, v_right > 7000 ? 7000 : v_right);
    // MotorData.Set_Speed_M1 = v_left;
    // MotorData.Set_Speed_M2 = v_right;

    //-----------------------------------------------
    
    // double treshhold = 1;
    // if (abs(gamma - kalman_a) < treshold ) // if it finnished rotating / aiming at end position -> drive forward
    // {
    //     if(abs(kalman_x-end_x) < treshhold && abs(kalman_y-end_y) < treshhold ) // if at end position, stop
    //     {
    //         MotorData.Set_Speed_M1 = 0;
    //         MotorData.Set_Speed_M2 = 0;
    //         std::cout << "stopped at end position" << std::endl;
    //     {
    //     else
    //     {
    //         MotorData.Set_Speed_M1 = 300;
    //         MotorData.Set_Speed_M2 = -300;
    //         std::cout << "drive forward" << std::endl;
    //     }

       
    // }
    // else if(gamma > 0 ) //if we want to rotate to positive angle , then drive with left wheel to rotate left
    // {
    //     MotorData.Set_Speed_M1 = 300; // maybe change these, dont know which way it will rotate with these settings before trying
    //     MotorData.Set_Speed_M2 = 300;
    //     std::cout << "should rotate left " << std::endl;
    // }
    // else //if we want to rotate to negative angle , then drive with right wheel to rotate right
    // {
    //     MotorData.Set_Speed_M1 = -300;
    //     MotorData.Set_Speed_M2 = -300;
    //     std::cout << "should rotate right" << std::endl;
    // }
    
    //-----------------------------------------------
    
    // if (delta < -5) {
    //     MotorData.Set_Speed_M1 = 300;
    //     MotorData.Set_Speed_M2 = -100;
    // } else if (delta > 5) {
    //     MotorData.Set_Speed_M1 = 100;
    //     MotorData.Set_Speed_M2 = -300;
    // } else {
    //     MotorData.Set_Speed_M1 = 300;
    //     MotorData.Set_Speed_M2 = -300;
    // }

    //set speed of motors:
    // MotorData.Set_Speed_M1 = 0;
    // MotorData.Set_Speed_M2 = 0;
}
