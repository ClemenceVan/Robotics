#include "Motors.hpp"

void Motors::odometry(){
    std::pair<double, double> encoders = refreshEncoders();
    double current_enc_l = encoders.first;
    double current_enc_r = encoders.second;
    std::cout << "ENCODER values in odometry: l = " << current_enc_l << ", r = " << current_enc_r << std::endl;
    //std::cout << "odometry: encoder values: l = " << current_enc_l << ", r = " << current_enc_r << std::endl;

    //----Kinematic model of a differential drive robot: ------
    double C = PULSES_PER_REVOLUTION * 6.6 * gearbox_ratio * 4 / circumference_wheel; // C = Nbr of pulses per mm = pulses per revolution * gearbox ratio * 4 (counts per puls) /circumference wheel
    double dt = 0.05;//50 / 1000; // sample time
    double vl = -(current_enc_l - prev_enc_l) / (dt*C); //
    double vr = (current_enc_r - prev_enc_r) / (dt*C);

    double v = (vr+vl)/2;
    double w = (vr-vl)/wheel_base; 
    double L = v*dt;
    //----Movement in robot coordinate system:-----
    double dA = w*dt; // A = theta = angle
    double dX =  L * cos(dA/2);
    double dY = L * sin(dA/2);

    //std::cout << "odometry: dx = " << dX << ", dy = " << dY << ", da = " << dA << std::endl;
    
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

    // // std::cout << "Axya = " << Axya << std::endl;
    // // std::cout << "Cxya_old = " << Cxya_old << std::endl;
    // // std::cout << "Au = " << Au << std::endl;
    // // std::cout << "Cu = " << Cu << std::endl;

    positionMutex.lock();
    this->v = v;
    covariance = Axya*Cxya_old*Axya.transpose() + Au*Cu*Au.transpose(); //Cxya_new in matlab code
    
// std::cout << "covariance odometry: " << covariance << std::endl;

    /* create position in global coordinate system */
    posX = prevPosX + dX*cos(prevPosA) + dY*sin(prevPosA);
    posY = prevPosY + dY*cos(prevPosA) - dX*sin(prevPosA);
    posA = fmod(prevPosA + dA,2*M_PI);
    // std::cout << "odometry: prevPosA = " << prevPosA << ", dA = " << dA << ", total posA = " << posA << std::endl;
    //updatePosition(posX, posY, posA, covariance); // maybe remove this ? odometry was working before adding this

    
    // std::cout << "Odometry Position: x = " << posX << ", y = " << posY << ", angle = " << posA << std::endl;
    /* update values */
    prevPosX = posX;
    prevPosY = posY;
    prevPosA = posA;
    Cxya_old = covariance;
    std::cout << posX << " " << posY << " " << posA << std::endl;
    positionMutex.unlock();
    prev_enc_l = current_enc_l;
    prev_enc_r = current_enc_r;
    
}