#include "./Robot.hpp"

void Robot::kalman() {
    /* --- cox parameters:------ */
    Eigen::MatrixXd sum_pf(3,3);
    sum_pf << lidar->getCovariance();
    //std::cout << "cox covariance: " << lidar->getCovariance() << std::endl;

    Eigen::VectorXd X_pf(3);
    X_pf << lidar->getPosX(), lidar->getPosY(), lidar->getPosA();


    /* --- odometry parameters:--- */
    Eigen::MatrixXd sum_Xi(3,3);
    sum_Xi = motors->getCovariance();
    //std::cout << "odometry covariance: " << motors->getCovariance() << std::endl;

    /* merged covariance matrix: */
    Eigen::MatrixXd Sum_plus_X(3,3); 
    Sum_plus_X << (sum_pf.inverse() + sum_Xi.inverse()).inverse(); // partial inverse instead .inverse() = .completeOrthogonalDecomposition().pseudoInverse()

    //std::cout << " kalman merged covariance: " << Sum_plus_X << std::endl;
    Eigen::VectorXd Xminus_i(3);
    Xminus_i << motors->getPosX(), motors->getPosY(), motors->getPosA();

    /* --- kalman filter:--- */
    Eigen::VectorXd Xplusi(3);
    Xplusi = sum_pf * (sum_pf + sum_Xi).inverse() * Xminus_i + sum_Xi * (sum_pf + sum_Xi).inverse() * X_pf;

    posX = Xplusi(0);
    posY = Xplusi(1);
    posA = Xplusi(2);
    
    /* --- update positions:--- */
    motors->updatePosition(posX, posY, posA, Sum_plus_X);
    lidar->updatePosition(posX, posY, posA);

    // std::cout << "kalman: x = " << posX << ", y = " << posY << ", angle = " << posA << std::endl;
}