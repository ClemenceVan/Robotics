#include "./Robot.hpp"
#include <chrono>
#include "servo.hpp"
#define ever ;;

using namespace std::chrono_literals; // This brings in the necessary literals like 500ms

Robot::Robot(Arena *arena, bool display): arena(arena), kalman_stream("kalman.txt") {
    this->posX = std::get<0>(arena->getOrigin());
    this->posY = std::get<1>(arena->getOrigin());
    wiringPiSetup(); 
    wiringPiSPISetup(SPI_Channel, 1000000);

    if (display)
        this->display = new Display(this->arena);

    this->camera = new Camera();
    this->lidar = new Lidar(*arena, display ? this->display : nullptr, PATH);
    this->motors = new Motors(*arena);
    _running = true;
    Servo(0,0);
}

Robot::~Robot() {
    // TODO: delete all the threads
    _running = false;
    // loopTh.join();
    motorsTh.join();
    // displayTh.join();
    batteryTh.join();
    kalman_stream.close();

    up();

    delete this->lidar;
    delete this->motors;
    // delete this->display;
}

void Robot::start() {
    motors->setSpeed(0, 0);
    camera->start();
    
    /* Motors thread */
    this->motorsTh = std::thread([&]() {
        while (_running) {
            motors->odometry();
            std::this_thread::sleep_for(500ms);
        }
    });
    this->motorsTh.detach();

    this->batteryTh = std::thread([&]() {
        INA219 Ina(0.01, 3.2);
        Ina.configure(RANGE_16V, GAIN_8_320MV, ADC_12BIT, ADC_12BIT);
        while(_running) {
            float current = Ina.current();
            float Volt = Ina.supply_voltage();
            std::cout << "current voltage : " << Volt << std::endl;
            if (Volt < 10.5) {
                std::cout << "\u001b[5m\u001b[40m\x1B[31m\u001b[1m💀💀 ERROR: voltage below 10.5 💀💀\x1B[37m\u001b[0m\u001b[35m\n" << std::endl;
                exit(1);
            } else if (Volt < 10.8)
                    while(true) std::cout << "\u001b[5m\u001b[40m\x1B[33m\u001b[1m👉👈 WARNING: voltage below 10.8 🥺 \x1B[37m\u001b[0m\u001b[35m" << std::endl;
            sleep(5);
        }
    });
    this->batteryTh.detach();
}

void Robot::loop() {
    bool arrived = false;
    int dropped = 0;
    while (dropped < 2) {
        camera->setGrabbed(false); // Clear grabbed flag
        while(!camera->grabbed()) {
            /* While we don't see a block, roam in the arena */
            camera->printMode("discovery");
            discover();
            /* Block has been found, picking it up */
            camera->printMode("pickup");
            // std::cout << "camera->getWorldDistance = " << camera->getWorldDistance() << std::endl;
            while (camera->getDistance() && !camera->grabbed()) {
                velocity_profile(arena->getOrigin().first, arena->getOrigin().second, 90*M_PI / 180, camera->getDistance());
                std::this_thread::sleep_for(200ms);
            }
            std::cout << "camera->getWorldDistance() = " << camera->getWorldDistance() << std::endl;
            
            // if (camera->grabbed())
            //     camera->setBlocksGrabbed(camera->getBlocksGrabbed() + 1);
            // std::cout << "Blocks grabbed : " << camera->getBlocksGrabbed() << std::endl;
            
            // start servo
            // std::cout << "starting servo" << std::endl;

            // for (int i_servo = 0; i_servo <= 180; i++)
            // {
            //     i_servo = (i_servo + 1) % 180;
            //     Servo(0, i_servo);
            //     delay(10);
            // }
            // std::cout << "ending servo" << std::endl;
        }
        camera->setGrabbed(false);
        motors->setSpeed(1000, 1500);
        std::this_thread::sleep_for(200ms);
        /* Block has been picked up, going home */
        camera->printMode("going home");
        std::cout << "Going home" << std::endl;

        double distance_to_home = sqrt(pow(arena->getOrigin().first - this->posX,2)  + pow(arena->getOrigin().second - this->posY,2));
        std::cout << "distance_to_home = " << distance_to_home << std::endl;
        double rotate_speed = 400;
        if(distance_to_home >= 0 ) {
            std::cout << "rotated to -90 (distance to home >= 150 cm)" << std::endl;
            rotate(-90,rotate_speed);
            std::this_thread::sleep_for(1000ms);

        } else if(this->posX >= arena->getSize().first - 60) {
                std::cout << "rotated to 180" << std::endl;
            rotate(180,rotate_speed);
            std::this_thread::sleep_for(1000ms);
            
        } else if(this->posX <= 60 ) {
                std::cout << "rotated to 0" << std::endl;
            rotate(0,rotate_speed);
            std::this_thread::sleep_for(1000ms);
            
        } else if(this->posY >= arena->getSize().second -60) {
                std::cout << "rotated to -90" << std::endl;
            rotate(-90,rotate_speed);
            std::this_thread::sleep_for(1000ms);
            
        }
        std::this_thread::sleep_for(200ms);
        while ((posX > arena->getOrigin().first + 33 || posX < arena->getOrigin().first - 33) || posY > 35) {
            velocity_profile(arena->getOrigin().first, 25,0);
            std::this_thread::sleep_for(100ms); 
        }
        /* Robot has brought block to origin, reverse motors */
        camera->printMode("reverse");
        std::cout << "arrived at origin" << std::endl;
        this->reverse();
        /* Block has been dropped, rotating away from dropped block */
        camera->printMode("rotating");
        std::cout << "rotating away from dropped blue box" << std::endl;
        rotate(90,rotate_speed);
        std::cout << "go fetch another block" << std::endl;
        // std::cout << "camera->grabbed() = " << camera->grabbed() << std::endl;
        std::cout << "camera->getBlocksGrabbed() = " << camera->getBlocksGrabbed() << std::endl;
        camera->setBlocksGrabbed(0);
        dropped++;
        down();
        /* Block successfully dropped at origin area */
    }
}

void Robot::run() {
    // motors->setSpeed(1000, 1050);
    down();
    /* Wait for lidar to be ready */
    while (!lidar->isDataReady());

    /* Loop thread */
    this->loopTh = std::thread(&Robot::loop, this);
    this->loopTh.detach();

    /* Kalman filter */
    int objX = arena->getSize().first / 2;
    int objY = arena->getSize().second / 2;
    while (_running) {
        while(!lidar->isDataReady());
        bool cox = lidar->cox_linefit();
        double cX = lidar->getPosX();
        double cY = lidar->getPosY();
        double cA = lidar->getPosA();

        double oX = motors->getPosX();
        double oY = motors->getPosY();
        double oA = motors->getPosA();
        if (!cox) {
            this->posX = oX;
            this->posY = oY;
            this->posA = oA;
        } else
            this->kalman();
        display->drawCoordinates(cX, cY, cA, oX, oY, oA, posX, posY, posA);
        camera->printCoordinates(cX, cY, cA * 180 / M_PI);
    }
}

bool Robot::velocity_profile(double end_x, double end_y, double end_a, double distance) {
    static int rev = 0;
    double v_new = 0;
    double w_new = 0;
    double v_right = 0;
    double v_left = 0;
    double k_gamma = 0;
    
    if (camera->shouldReverse()) {
        this->reverse();
        camera->setReverse(false);
        rev = 5;
    }
    if (isnan(distance)) { // kalman in use
        // std::cout << "kalman in use! " << std::endl;
        double dx = end_x - posX;
        double dy = end_y - posY;
        double kalman_a = fmod(posA ,2*M_PI); //posA + 90*PI/180

        double epsilon = 0;
        if(dx <= 0.1 && dx >= 0)
            dx = 1;
        epsilon = atan2(dy,dx);

        double d = sqrt(pow(dy,2)+pow(dx,2)); // distance to drive when we have turned;
        double gamma = epsilon - kalman_a; // the angle we should turn to in global coordinate system to aim at new position
        
        if (gamma >= M_PI)
            gamma = gamma - 2 * M_PI;
        else if (gamma <= -M_PI)
            gamma = gamma + 2 * M_PI;
        double delta = end_a - gamma - kalman_a; // desired angle in world //end_a - epsilon
        double k_rho = this->rho;
        k_gamma = this->gamma;

        double rho = d;
        v_new = k_rho * rho;
        w_new = k_gamma * gamma ; //+ k_delta * delta;
        std::cout << "w_new in kalman before obstacle avoidance :" << w_new << std::endl;
        w_new += camera->getObstacleDistance() / (this->delta / 2);
        std::cout << "w_new in kalman after obstacle avoidance:" << w_new << std::endl;
        if (d >= 0 && d < 20) { //&& fabs(end_a - kalman_a) <= 0.1) {
            v_new = 0;
            // std::cout << "open servo" << std::endl;
            // up();
            return true;
        }
    } else {
        std::cout << "distance = " << distance << std::endl;
        v_new = this->c1;
        w_new = this->c2 * distance / 10 ; // cm
        std::cout << "w_new in camera before obstacle avoidance :" << w_new << std::endl;
        w_new += camera->getObstacleDistance() / this->delta;
        std::cout << "w_new in camera after obstacle avoidance :" << w_new << std::endl;
    }
    if (rev > 0) {
        w_new += camera->getObstacleDistance();
        rev--;
    }

    v_right = (v_new + w_new * motors->wheel_base / 2);
    v_left = (v_new - w_new * motors->wheel_base / 2);

    std::cout << "v_left = " << v_left << ", v_right = " << v_right << std::endl;
    motors->setSpeed(v_left > 7000 ? 7000 : v_left, v_right > 7000 ? 7000 : v_right);

    return false;
}

void Robot::discover() {
    std::cout << "inside discover" << std::endl;
    double middleX = arena->getSize().first / 2;
    double middleY = arena->getSize().second - 150;

    while (camera->getDistance() == 0) {
        while (camera->getDistance() == 0 && !velocity_profile(middleX, middleY, 0))
            std::this_thread::sleep_for(500ms);
        while (camera->getDistance() == 0 && !velocity_profile(arena->getOrigin().first, arena->getOrigin().second + 50, 0))
            std::this_thread::sleep_for(500ms);
    }
}

void Robot::rotate(double desired_angle, double angle_velocity) {
    // motors->setSpeed(angle_velocity, -angle_velocity);
    desired_angle = desired_angle * M_PI / 180;
    double angle_difference = desired_angle - this->motors->getPosA();
    if (angle_difference < 0) {
        std::cout << "rotate right" << std::endl;
        motors->setSpeed(angle_velocity, -angle_velocity);
    } else if(angle_difference > 0) {
        std::cout << "rotate left" << std::endl;
        motors->setSpeed(-angle_velocity, angle_velocity);
    } else {
        std::cout << "angle diff = 0, return" << std::endl;
        return;
    }

    // while (abs(this->motors->getPosA() - desired_angle * M_PI / 180) >= 40 * M_PI / 180)
    
    for (ever) {
        std::cout << "original angle diff = " << angle_difference << std::endl;
        std::cout << "current angle diff = " << desired_angle - this->motors->getPosA() << std::endl;
        std::this_thread::sleep_for(500ms);
        std::cout << "ROTATING: this->motors->getPosA() = " << this->motors->getPosA() << std::endl;
        std::cout << "desired_angle = " << desired_angle << std::endl;
        std::cout << "this->motors->getPosA() < desired_angle + 20 * M_PI / 180 = " << (this->motors->getPosA() < desired_angle + 20 * M_PI / 180) << std::endl;
        std::cout << "this->motors->getPosA() > desired_angle - 20 * M_PI / 180 = " << (this->motors->getPosA() > desired_angle - 20 * M_PI / 180) << std::endl;
        
        if (this->motors->getPosA() < desired_angle + 10 * M_PI / 180 && this->motors->getPosA() > desired_angle - 10 * M_PI / 180) {
            std::cout << "stop rotating: angle diff < 20, return" << std::endl;
            motors->setSpeed(0, 0);
            return;
        }

        if (abs(desired_angle - this->motors->getPosA()) < 40*M_PI/180) {
            std::cout << " OTHER IF statemnt : angle diff < 40 deg, return" << std::endl;
            motors->setSpeed(0, 0);
            return;
        }
    }
}

void Robot::reverse() {
    motors->setSpeed(-2000, -2000);
    // double homeX = motors->getPosX();
    // double homeY = motors->getPosY();
    double homeX = posX;
    double homeY = posY;
    while ((posX > homeX - 5 && posX < homeX + 5) && (posY > homeY - 5 && posY < homeY + 5))
        std::this_thread::sleep_for(500ms);
}