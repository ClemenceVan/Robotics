#include "./Robot.hpp"
#include <chrono>
using namespace std::chrono_literals; // This brings in the necessary literals like 500ms
Robot::Robot(Arena *arena, bool display): arena(arena), kalman_stream("kalman.txt") {
    this->posX = std::get<0>(arena->getOrigin());
    this->posY = std::get<1>(arena->getOrigin());
    
    #ifndef _WIN32
        wiringPiSetup(); 
        wiringPiSPISetup(SPI_Channel, 1000000);
    #endif

    if (display)
        this->display = new Display(this->arena);

    this->lidar = new Lidar(*arena, display ? this->display : nullptr, PATH);
    this->motors = new Motors(*arena);
    this->camera = new Camera();
    _running = true;
}

Robot::~Robot() {
    // TODO: delete all the threads
    _running = false;
    // lidarTh.join();
    motorsTh.join();
    // displayTh.join();
    batteryTh.join();
    kalman_stream.close();
  

    delete this->lidar;
    delete this->motors;

    // delete this->display;
}

void Robot::start() {
    motors->setSpeed(0, 0);
    camera->start();

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
        const float SHUNT_OHMS = 0.01;
        const float MAX_EXPECTED_AMPS = 3.2;
        const float Delay_ms = 1000.0;
        INA219 Ina(SHUNT_OHMS, MAX_EXPECTED_AMPS);
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

void Robot::run() {
    // motors->setSpeed(1000, 1050);

    
    while (!lidar->isDataReady());

    /* Lidar thread */
    this->lidarTh = std::thread([this]() {
        bool arrived = false;
        bool neverGoBack = false;
        while (_running) {
            while(camera->getBlocksGrabbed() < 3) {
                /* While we don't see a block, roam in the arena */
                camera->printMode("discovery");
                discover();
                /* Block has been found, picking it up */
                camera->printMode("pickup");
                // std::cout << "camera->getWorldDistance = " << camera->getWorldDistance() << std::endl;
                while (!velocity_profile(arena->getOrigin().first, arena->getOrigin().second, 90*M_PI / 180, camera->getDistance())) {
                    std::this_thread::sleep_for(500ms);
                    neverGoBack = true;
                }
                if (camera->grabbed())
                    // i--;
                    // continue;
                    camera->setBlocksGrabbed(camera->getBlocksGrabbed() + 1);
                std::cout << "Blocks grabbed : " << camera->getBlocksGrabbed() << std::endl;
            }
            /* Block has been picked up, going home */
            camera->printMode("going home");
            std::cout << "Going home" << std::endl;
            while ((posX > arena->getOrigin().first + 17 || posX < arena->getOrigin().first - 17) || posY > 30) {
                // std::cout << "X check : " << posX << " Y check : " << posY << std::endl;
                velocity_profile(arena->getOrigin().first, 15,0);
                std::this_thread::sleep_for(100ms); 
                
            }
            /* Robot has brought block to origin, reverse motors */
            camera->printMode("reverse");
            std::cout << "arrived at origin" << std::endl;
            motors->setSpeed(-1000, -1000);
            double homeX = motors->getPosX();
            double homeY = motors->getPosY();
            while ((posX > homeX - 5 || posX < homeX + 5) && posY < homeY + 5) {
                std::this_thread::sleep_for(500ms);
            }
            camera->printMode("rotating");
            std::cout << "rotating away from dropped blue box" << std::endl;
            motors->setSpeed(-500, 500);
             
            while(fabs(this->posA - 90*M_PI/180)>= 20*M_PI/180) {
                 std::this_thread::sleep_for(50ms);
                 std::cout << "fabs(this->posA - 90*M_PI/180) " << fabs(this->posA - 90*M_PI/180) << std::endl;
            }
            motors->setSpeed(0, 0);
            /* Block successfully dropped at origin area */

            //while (posY > arena->getOrigin().second + 10 && posX > arena->getOrigin().first+ 10 && posX < arena->getSize().first - 10) {
            //while (posX > arena->getOrigin().first + 10 && posY > arena->getOrigin().second+ 10 && posY < arena->getSize().second - 10) {
       }
    });
    this->lidarTh.detach();
    /* *** */


    int objX = arena->getSize().first / 2;
    int objY = arena->getSize().second / 2;
    while (_running) {
        // sleep(1);
        // while(!lidar->cox_linefit());
        while(!lidar->isDataReady());
        // std::this_thread::sleep_for(2000ms);
        bool cox = lidar->cox_linefit();
        double cX = lidar->getPosX();
        double cY = lidar->getPosY();
        double cA = lidar->getPosA();

        double oX = motors->getPosX();
        double oY = motors->getPosY();
        double oA = motors->getPosA();
        // double cX = oX;
        // double cY = oY;
        // double cA = oA;
        //     this->posX = cX;
        //     this->posY = cY;
        //     this->posA = cA;

        if (!cox) {
            this->posX = oX;
            this->posY = oY;
            this->posA = oA;
        } else
            this->kalman();
        // posA += 90 * M_PI / 180;
            // std::this_thread::sleep_for(500ms);
        display->drawCoordinates(cX, cY, cA, oX, oY, oA, posX, posY, posA);
        camera->printCoordinates(cX, cY, cA * 180 / M_PI);
        // motors->velocity_profile(100.5, 34, 0);
        // motors->velocity_profile(22, 203, -90*M_PI/180);
        // velocity_profile(objX, objY, 0); // 143 = y, 
        // velocity_profile(117, 65, 90*M_PI / 180, camera->getDistance());
        //motors->setSpeed(400, 400);
            // motors->getPosX(),motors->getPosY(), motors->getPosA(),
            // lidar->getPosX(), lidar->getPosY(), lidar->getPosA());
    }
}

bool Robot::velocity_profile(double end_x, double end_y, double end_a, double distance) {
    // positionMutex.lock();
    
    //double rho = total_v;
    //std::cout << "rho = " << rho << std::endl;
    double v_new = 0;
    double w_new = 0;
    double v_right = 0;
    double v_left = 0;
    bool arrived = false;
    if (isnan(distance)) { // kalman in use
        // std::cout << "kalman in use! " << std::endl;
        double dx = end_x - posX;
        double dy = end_y - posY;
        double kalman_a = fmod(posA ,2*M_PI); //posA + 90*PI/180
        // positionMutex.unlock();
       //std::cout << "velocity profile dx = " << dx << ", dy = " << dy << std::endl;

        double epsilon = 0;
        if(dx <= 0.1 && dx >= 0)
            dx = 1;
        epsilon = atan2(dy,dx); // be careful of dx close to zero ?
        
        // std::cout << "epsilon = " << epsilon << std::endl;

        double d = sqrt(pow(dy,2)+pow(dx,2)); // distance to drive when we have turned;



        
        // if(d <=10)
        // {
        //     std::cout << "close to endpoint, robot should stop UwU" << std::endl;
        //     return true;
        //     //setSpeed(0,0);
        // }
        // std::cout << "d = " << d << std::endl;
        // print kalman-
        // epsilon = 1.45917 ~ 90 deg
        // gamma = 1.62135  ~ 90 deg this should be 0
        // delta = -1.45917 ~ 90  degr should be 0
        // std::cout << "kalman_a = " << kalman_a << std::endl;
        double gamma = epsilon - kalman_a; // the angle we should turn to in global coordinate system to aim at new position
        
        
        // if ((dx < 0 && dy > 0) || (dx < 0 && dy < 0))
        //     epsilon = epsilon + M_PI;
        
        if (gamma >= M_PI)
            gamma = gamma - 2 * M_PI;
        else if (gamma <= -M_PI)
            gamma = gamma + 2 * M_PI;
        // std::cout << "gamma = " << gamma << std::endl;


        double delta = end_a - gamma - kalman_a; // desired angle in world //end_a - epsilon
                                                 // std::cout << "delta = " << delta << std::endl;

        // double threshold = 0.2;
        // std::cout << "abs(gamma - kalman_a) = " << abs(epsilon - kalman_a) << std::endl;
        // std::cout << "kalman_a = " << kalman_a << std::endl;
        // if (abs(epsilon - kalman_a) < threshold) // DRIVE FORWARD: if it finnished rotating / aiming at end position -> drive forward
        // {
        //     // if (abs(kalman_x - end_x) < threshold && abs(kalman_y - end_y) < threshold) // if at end position, stop
        //     if(d >= 0 && d < 30)
        //     {
        //         v_left = 0;
        //         v_right = 0;
        //         std::cout << "stopped at end position" << std::endl;
        //         arrived = true;
        //     }
        //     else
        //     {
        //         v_left = 1000;
        //         v_right = 1000;
        //         std::cout << "drive forward" << std::endl;
        //     }
        // }
        // else if (gamma > 0) // if we want to rotate to positive angle , then drive with left wheel to rotate left
        // {
        //     v_left = -500; // maybe change these, dont know which way it will rotate with these settings before trying
        //     v_right = 500;
        //     std::cout << "should rotate left " << std::endl;
        // }
        // else // ROTATE : if we want to rotate to negative angle , then drive with right wheel to rotate right
        // {
        //     v_left = 500;
        //     v_right = -500;
        //     std::cout << "should rotate right" << std::endl;
        // }

        // v_new = d * this->c1;
        // w_new = this->c2 * gamma * 100;
        // v_right = (v_new + w_new * motors->wheel_base / 2);
        // v_left = (v_new - w_new * motors->wheel_base / 2);


        // double k_rho = 10;
        // double k_gamma = 5;
        // double k_delta = 0.1; // can change this value
        double k_rho = this->rho;
        double k_gamma = this->gamma;
        // std::cout << "k_rho = " << k_rho << ", k_gamma = " << k_gamma << ", k_delta = " << k_delta << std::endl;

        double rho = d; // ?????? dont know value of this   // whst is
        v_new = k_rho * rho;
        w_new = k_gamma * gamma ; //+ k_delta * delta;
        if (d >= 0 && d < 20)
        { //&& fabs(end_a - kalman_a) <= 0.1) {
            v_new = 0;
            arrived = true;
            return true;
        }
    }
    else
    {
        std::cout << "distance = " << distance << std::endl;
        // std::cout << " camera->grabbed() = " << camera->grabbed() << std::endl;
        if (camera->getWorldDistance() < 15 || camera->grabbed())
        {
            std::cout << "camera->getWorldDistance() < 15 || camera->grabbed() == TRUE" << std::endl;
            return true;
        }
            
        v_new = this->c1;
        w_new = this->c2 * distance / 10 ; // cm
        // w_new = this->c2 * distance / 10 ; // cm
        // v_right = (v_new + w_new * motors->wheel_base / 2);
        // v_left = (v_new - w_new * motors->wheel_base / 2);
    }

    // std::cout << "v_new = " << v_new << std::endl;
    // std::cout << "w_new = " << w_new << std::endl;

   
    std::cout << "camera->getObstacleDistance() = " << camera->getObstacleDistance() << std::endl;
    w_new += camera->getObstacleDistance() / this->delta;
    v_right = (v_new + w_new * motors->wheel_base / 2);
    v_left = (v_new - w_new * motors->wheel_base / 2);

    std::cout << "v_left = " << v_left << ", v_right = " << v_right << std::endl;

    motors->setSpeed(v_left > 7000 ? 7000 : v_left, v_right > 7000 ? 7000 : v_right);
    // MotorData.Set_Speed_M1 = v_left;
    // MotorData.Set_Speed_M2 = v_right;

    //-----------------------------------------------

    return arrived;
}

void Robot::discover()
{
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
