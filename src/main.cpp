#include "./Robot/Robot.hpp"
#include "./Robot/Arena.hpp"
#include "pch.h"

int timeOffset = 0;

int main(int ac, char **av) {
    if (ac < 5) {
        std::cerr << "Usage: ./robot [rho] [gamma] [delta] [c1] [c2]" << std::endl;
        return 1;
    }
    
    // Robot robot(new Arena({120, 242}, {60, 30}), true); // arena
      Robot robot(new Arena({240, 363}, {120,39}), true); // arena
    // Robot robot(new Arena({130, 260}, {65, 15}), true); // room

    robot.debug(std::stod(av[1]), std::stod(av[2]), std::stod(av[3]), std::stod(av[4]), std::stod(av[5]));
    std::cout << "Starting robot" << std::endl;
    robot.start();
    timeOffset = std::time(nullptr);
    robot.run();
    return 0;
}