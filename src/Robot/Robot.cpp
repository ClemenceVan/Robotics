#include "./Robot.hpp"
#include <chrono>
using namespace std::chrono_literals; // This brings in the necessary literals like 500ms
Robot::Robot(Arena *arena, bool display): arena(arena) {
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
    _running = true;
}

Robot::~Robot() {
    // TODO: delete all the threads
    _running = false;
    // lidarTh.join();
    motorsTh.join();
    // displayTh.join();
    batteryTh.join();

    delete this->lidar;
    delete this->motors;
    // delete this->display;
}

void Robot::start() {
    /* Lidar thread */
    this->lidarTh = std::thread([&]() {
        while (!lidar->isDataReady());
        while (_running) {
            lidar->cox_linefit();
            // sleep(1);
            lidar->waitForUpdate();
        }
    });
    this->lidarTh.detach();
    /* *** */

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
    motors->setSpeed(1400, 1500);
    int objX = arena->getSize().first / 2;
    int objY = arena->getSize().second / 2;
    while (!lidar->isDataReady());
    while (_running) {
        // sleep(1);
        // while(!lidar->cox_linefit());
        lidar->cox_linefit();
        double cX = lidar->getPosX();
        double cY = lidar->getPosY();
        double cA = lidar->getPosA();

        double oX = motors->getPosX();
        double oY = motors->getPosY();
        double oA = motors->getPosA();

        this->posX = cX;
        this->posY = cY;
        this->posA = cA;
            // sleep(1);
            // lidar->waitForUpdate();/
        this->kalman();
        //posA += 90 * M_PI / 180;
            std::this_thread::sleep_for(500ms);
        display->drawCoordinates(cX, cY, cA, oX, oY, oA, posX, posY, posA);
        // motors->velocity_profile(100.5, 34, 0);
        motors->velocity_profile(22, 203, -90*M_PI/180);
        // motors->velocity_profile(objX, objY, 0);
        
            // motors->getPosX(),motors->getPosY(), motors->getPosA(),
            // lidar->getPosX(), lidar->getPosY(), lidar->getPosA());
    }
}