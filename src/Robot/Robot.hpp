#include "../include.hpp"

#include "Arena.hpp"
#include "../Lidar/Lidar.hpp"
#include "../Motors/Motors.hpp"
#include "Display.hpp"

#ifdef _WIN32 // if on windows, use file as data source to simulate lidar
    #define PATH "../testfile90.txt"
#else
    #define PATH ""
#endif

class Robot {
    private:
    /* Coordinates */
    double posX = 0;
    double posY = 0;
    double posA = 0;
    /* Modules */
    Arena *arena;
    Lidar *lidar;
    Motors *motors;
    Display *display;
    /* Threads */
    std::thread lidarTh;
    std::thread motorsTh;
    public:
    Robot(Arena *arena): arena(arena) {
        this->posX = std::get<0>(arena->getOrigin());
        this->posY = std::get<1>(arena->getOrigin());

        this->lidar = new Lidar(*arena, false, PATH);
        this->motors = new Motors(*arena);
        // this->display = Display({100, 100}, arena);

    }

    ~Robot() {
        // destroy threads eventually
    }

    void start() {
        /* Lidar thread */
        this->lidarTh = std::thread([&]() {
            while (!lidar->isDataReady());
            while (true) {
                lidar->cox_linefit();
                sleep(1);
                lidar->waitForUpdate();
            }
        });
        this->lidarTh.detach();
        /* *** */

        /* Motors thread */
        this->motorsTh = std::thread([&]() {
            while (true) {
                motors->odometry();
                sleep(0.5);
            }
        });
        this->motorsTh.detach();
    }

    void run() {
        while (true) {
            sleep(1);
            this->kalman();
        }
    }

    void kalman() {
        /* --- cox parameters:------ */
        Eigen::MatrixXd sum_pf(3,3);
        sum_pf << lidar->getCovariance();

        Eigen::VectorXd X_pf(3);
        X_pf << lidar->getPosX(), lidar->getPosY(), lidar->getPosA();

        /* --- odometry parameters:--- */
        Eigen::MatrixXd sum_Xi(3,3);
        sum_Xi = motors->getCovariance();

        Eigen::MatrixXd Sum_plus_X(3,3);
        Sum_plus_X << (sum_pf.inverse() + sum_Xi.inverse()).inverse();

        Eigen::VectorXd Xminus_i(3);
        Xminus_i << motors->getPosX(), motors->getPosY(), motors->getPosA();

        /* --- kalman filter:--- */
        Eigen::VectorXd Xplusi(3);
        Xplusi = sum_pf * (sum_pf + sum_Xi).inverse() * Xminus_i + sum_Xi * (sum_pf + sum_Xi).inverse() * X_pf;

        posX = Xplusi(0);
        posY = Xplusi(1);
        posA = Xplusi(2);
        
        /* --- update positions:--- */
        motors->updatePosition(posX, posY, posA);
        lidar->updatePosition(posX, posY, posA);

        std::cout << "kalman: x = " << posX << ", y = " << posY << ", angle = " << posA << std::endl;
    }
};