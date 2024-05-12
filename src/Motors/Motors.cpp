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