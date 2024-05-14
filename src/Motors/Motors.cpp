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
    //posA = 90*M_PI/180;
    posA = 0;
    prevPosX = posX;
    prevPosY = posY;
    prevPosA = posA;

    // MotorDataType motor;
    // Send_Read_Motor_Data(&MotorData);
    
    // printf("\u001b[35m\n");
    
    Send_Read_Motor_Data(&MotorData);
    std::cout << "ENCODER VALUES: M1 = " << MotorData.Encoder_M1<< ", M2 = " <<MotorData.Encoder_M2<< std::endl;
    offset_m1 = MotorData.Encoder_M1;
    offset_m2 = -MotorData.Encoder_M2;

    this->readWriteTh = std::thread([this] {
        while(true) {
            usleep(100000);
            this->motorMutex.lock();
            Send_Read_Motor_Data(&MotorData);
            this->motorMutex.unlock();
        }
    });
    this->readWriteTh.detach();
}

std::pair<double, double> Motors::refreshEncoders() {
    this->motorMutex.lock();
    Send_Read_Motor_Data(&MotorData);
    std::pair<double, double> encoders = std::make_pair(MotorData.Encoder_M1 - offset_m1, -MotorData.Encoder_M2 - offset_m2);
    this->motorMutex.unlock();
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

void Motors::setSpeed(int left, int right) {
    this->motorMutex.lock();
    MotorData.Set_Speed_M1 = -left;
	MotorData.Set_Speed_M2 = right;
    Send_Read_Motor_Data(&MotorData);
    this->motorMutex.unlock();
}

/*void check_angle_boundary(double angle){
    if(angle >= M_PI) 
        return angle - 2*M_PI;
    el
 }
 */

void Motors::velocity_profile(double end_x, double end_y, double end_a)
{
    positionMutex.lock();
    double dx = end_x - posX;
    double dy = end_y - posY;
    double kalman_a = fmod(posA +90*M_PI/180 ,2*M_PI); // 
    double total_v = this->v;
    positionMutex.unlock();
    //std::cout << "velocity profile dx = " << dx << ", dy = " << dy << std::endl;

    double epsilon = 0;
    if(dx <= 0.1 && dx >= 0)
        dx = 1;
    epsilon = atan2(dy,dx); // be careful of dx close to zero ?
    
    //std::cout << "epsilon = " << epsilon << std::endl;

    double d = sqrt(pow(dy,2)+pow(dx,2)); // distance to drive when we have turned;

    if(d <= 3)
    {
        std::cout << "close to endpoint, robot should stop UwU" << std::endl;
        //setSpeed(0,0);
    }
    std::cout << "d = " << d << std::endl;
    //print kalman- 
    //epsilon = 1.45917 ~ 90 deg
    //gamma = 1.62135  ~ 90 deg this should be 0
    //delta = -1.45917 ~ 90  degr should be 0
   // std::cout << "kalman_a = " << kalman_a << std::endl;
    double gamma = epsilon - kalman_a; // the angle we should turn to in global coordinate system to aim at new position
    //std::cout << "gamma = " << gamma << std::endl;

    double delta = end_a - gamma - kalman_a; // desired angle in world //end_a - epsilon
    //std::cout << "delta = " << delta << std::endl;

    // double k_rho = 10;
    // double k_gamma = 5; 
    // double k_delta = 0.1; // can change this value
    double k_rho = this->rho;
    double k_gamma = this->gamma;
    double k_delta = this->delta;
    //std::cout << "k_rho = " << k_rho << ", k_gamma = " << k_gamma << ", k_delta = " << k_delta << std::endl;

    double rho = d; // ?????? dont know value of this   // whst is
    //double rho = total_v;
    //std::cout << "rho = " << rho << std::endl;

    double v_new = k_rho*rho;
    //std::cout << "v_new = " << v_new << std::endl;

    double w_new = k_gamma * gamma + k_delta * delta;
    //std::cout << "w_new = " << w_new << std::endl;

    if (d >= 0 && d < 3 ){//&& fabs(end_a - kalman_a) <= 0.1) {
        std::cout << "d < 3, robot should rotate UwU" << std::endl;
        v_new = 0;
    }
    double v_right =( v_new + w_new * wheel_base / 2);
    double v_left = (v_new - w_new*wheel_base / 2) ;
    // double v_left =( v_new + w_new * wheel_base / 2);
    // double v_right= (v_new - w_new*wheel_base / 2) ;

    std::cout << "v_left = " << v_left << ", v_right = " << v_right << std::endl;
    
    setSpeed(v_left > 7000 ? 7000 : v_left, v_right > 7000 ? 7000 : v_right);
    // MotorData.Set_Speed_M1 = v_left;
    // MotorData.Set_Speed_M2 = v_right;

    //-----------------------------------------------
    
   
}
