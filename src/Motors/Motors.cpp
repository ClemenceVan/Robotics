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

Motors::Motors(Arena arena): arena(arena), odometry_stream("odometry_file.txt") {
    posX = arena.getOrigin().first;
    posY = arena.getOrigin().second;
    posA = 90*M_PI/180;
    //posA = 0;
    prevPosX = posX;
    prevPosY = posY;
    prevPosA = posA;

    // MotorDataType motor;
    // Send_Read_Motor_Data(&MotorData);
    
    // printf("\u001b[35m\n");
    this->readWriteFlag = false;
    MotorData.Set_Speed_M1 = 0;
    MotorData.Set_Speed_M2 = 0;
    this->motorMutex.lock();
    this->refreshEncoders();
    this->motorMutex.unlock();
    std::cout << "ENCODER VALUES: M1 = " << MotorData.Encoder_M1<< ", M2 = " <<MotorData.Encoder_M2<< std::endl;
    offset_m1 = MotorData.Encoder_M1;
    // offset_m2 = -MotorData.Encoder_M2;
    offset_m2 = MotorData.Encoder_M2;
    std::cout << "Offset m1 = " << offset_m1 << ", offset m2 = " << offset_m2 << std::endl;

    this->readWriteTh = std::thread([this] {
        while(true) {
            // usleep(100000);
            std::this_thread::sleep_for(std::chrono::milliseconds(90));
            this->motorMutex.lock();
                // Send_Read_Motor_Data(&MotorData);
            if (!this->readWriteFlag)
                this->refreshEncoders();
            this->readWriteFlag = false;
            this->motorMutex.unlock();
        }
    });
    this->readWriteTh.detach();
}

std::pair<double, double> Motors::refreshEncoders() {
    // this->motorMutex.lock();
    this->readWriteFlag = true;
    Send_Read_Motor_Data(&MotorData);
    std::pair<double, double> encoders = std::make_pair(MotorData.Encoder_M1 - offset_m1, MotorData.Encoder_M2 - offset_m2);
    this->enc_l = encoders.first;
    this->enc_r = encoders.second;
    // this->motorMutex.unlock();
    return encoders;
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
    // positionMutex.lock();
    return posX;
    // positionMutex.unlock();
}

double Motors::getPosY() {
    // positionMutex.lock();
    return posY;
    // positionMutex.unlock();
}

double Motors::getPosA() {
    // positionMutex.lock();
    return posA;
    // positionMutex.unlock();
}

Eigen::MatrixXd Motors::getCovariance() {
    // positionMutex.lock();
    return covariance;
    // positionMutex.unlock();
}

std::pair<int, int> Motors::getSpeed() {
    // this->motorMutex.lock();
    return std::make_pair(speedL, speedR);
    // this->motorMutex.unlock();
}

void Motors::setSpeed(int left, int right) {
    // std::cout << "setspeed yeah" << std::endl;
    this->motorMutex.lock();
    this->speedL = left;
    this->speedR = right;
    MotorData.Set_Speed_M1 = -left;
	MotorData.Set_Speed_M2 = right;
    // this->refreshEncoders();
    this->motorMutex.unlock();
}


/*void check_angle_boundary(double angle){
    if(angle >= M_PI) 
        return angle - 2*M_PI;
    el
 }
 */
