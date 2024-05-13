#include "./Robot.hpp"

void Robot::kalman() {
    /* --- cox parameters:------ */
    Eigen::MatrixXd sum_pf(3,3);
    sum_pf << lidar->getCovariance();
    std::cout << "cox covariance: " << lidar->getCovariance() << std::endl;
    
    Eigen::VectorXd X_pf(3);
    std::cout << "lidar position BEGINNING of kalman: " << lidar->getPosX() << " , "<< lidar->getPosY() << " , "<< lidar->getPosA() << std::endl;
    std::cout << "odometry position BEGINNING of kalman: " <<  motors->getPosX() << " , " <<motors->getPosY() << " , " <<motors->getPosA() << std::endl;
    X_pf << lidar->getPosX(), lidar->getPosY(), lidar->getPosA();
 


    /* --- odometry parameters:--- */
    Eigen::MatrixXd sum_Xi(3,3);
    sum_Xi = motors->getCovariance();
    std::cout << "odometry covariance: " << motors->getCovariance() << std::endl;


    if(sum_pf(1,1)> sum_Xi(1,1)){
        std::cout << "WHAHAHQWHHWAHQHANHSDHAEWHFADSFASDHFKADSNVFAHDSFKNASKDVFNASRIKFNAKDSNVAJEFSGNNASKDVFNAJESGFNNA"<< std::endl;
    }

    /* merged covariance matrix: */
    Eigen::MatrixXd Sum_plus_X(3,3); 
    Sum_plus_X << (sum_pf.completeOrthogonalDecomposition().pseudoInverse() + sum_Xi.completeOrthogonalDecomposition().pseudoInverse()).completeOrthogonalDecomposition().pseudoInverse(); // partial inverse instead .inverse() = .completeOrthogonalDecomposition().pseudoInverse()
    
    //std::cout << " kalman merged covariance: " << Sum_plus_X << std::endl;
    Eigen::VectorXd Xminus_i(3);
    Xminus_i << motors->getPosX(), motors->getPosY(), motors->getPosA();

    /* --- kalman filter:--- */
    Eigen::VectorXd Xplusi(3);
    Xplusi = sum_pf * (sum_pf + sum_Xi).completeOrthogonalDecomposition().pseudoInverse() * Xminus_i + sum_Xi * (sum_pf + sum_Xi).completeOrthogonalDecomposition().pseudoInverse() * X_pf;

    posX = Xplusi(0);
    posY = Xplusi(1);
    posA = fmod(Xplusi(2),2*M_PI); //* 90 * M_PI / 180;

    //posA += 90 * M_PI / 180;
    
    //std::cout << "XXXXXXXXXXXXX kalman: x = " << posX << ", y = " << posY << ", angle = " << posA << std::endl;

    std::cout << "lidar position END of kalman: " << lidar->getPosX()<< " , " <<lidar->getPosY() << " , "<< lidar->getPosA() << std::endl;
    std::cout << "odometry position END of kalman: " <<  motors->getPosX()<< " , " <<motors->getPosY()<< " , " <<motors->getPosA() << std::endl;

    /* --- update positions:--- */
    motors->updatePosition(posX, posY, posA, Sum_plus_X);
    lidar->updatePosition(posX, posY, posA);

    
}