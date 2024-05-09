#include "../../../libraries/spi_com.h"

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

std::pair<double, double> Motors::refreshEncoders() {
    Send_Read_Motor_Data(&MotorData);
    return std::make_pair(-(MotorData.Encoder_M1) - offset_m1, MotorData.Encoder_M2 - offset_m2);
}

void Motors::updatePosition(double x, double y, double a) {
    positionMutex.lock();
    prevPosX = x; // right coordinates to update ?
    prevPosY = y;
    prevPosA = a;
    posX = x;
    posY = y;
    posA = a;
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