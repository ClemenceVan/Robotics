#include "./Robot.hpp"

Robot::Robot(Arena *arena): arena(arena) {
    this->posX = std::get<0>(arena->getOrigin());
    this->posY = std::get<1>(arena->getOrigin());

    this->lidar = new Lidar(*arena, false, PATH);
    this->motors = new Motors(*arena);
    // this->display = Display({100, 100}, arena);
}

Robot::~Robot() {
    // TODO: delete all the threads
    delete this->lidar;
    delete this->motors;
    // delete this->display;
}

void Robot::start() {
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

void Robot::run() {
    while (true) {
        sleep(1);
        this->kalman();
    }
}