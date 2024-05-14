#include "./Robot/Robot.hpp"
#include "./Robot/Arena.hpp"
#include "include.hpp"

int main(int ac, char **av) {
    if (ac != 4) {
        std::cerr << "Usage: ./robot [rho] [gamma] [delta]" << std::endl;
        return 1;
    }
    
    // Robot robot(new Arena({39, 54}, {39/2, 54/2}), true); // big box
    // Robot robot(new Arena({120, 242}, {60, 30}), true); // arena
    // Robot robot(new Arena({363, 240}, {178, 33}), true); // arena
    Robot robot(new Arena({123, 243}, {59, 34}), true); // subway surfer room
    robot.debug_velocity(std::stod(av[1]), std::stod(av[2]), std::stod(av[3]));
    robot.start();
    robot.run();
    return 0;
}
