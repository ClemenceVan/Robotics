#include "./Robot/Robot.hpp"
#include "./Robot/Arena.hpp"
#include "pch.h"

int timeOffset = 0;
int main(int ac, char **av) {
    timeOffset = std::time(nullptr);
    if (ac < 5) {
        std::cerr << "Usage: ./robot [rho] [gamma] [delta] [c1] [c2]" << std::endl;
        return 1;
    }
    
    // Robot robot(new Arena({39, 54}, {39/2, 54/2}), true); // big box
    // Robot robot(new Arena({120, 242}, {60, 30}), true); // arena
    //Robot robot(new Arena({240, 363}, {121, 39}), true); // arena
      Robot robot(new Arena({240, 363}, {120,39}), true); // arena
    // Robot robot(new Arena({130, 260}, {65, 15}), true); // room
    // std::cout << ac << std::endl;
    // std::cout << av[6] << std::endl;
    // if (ac == 7) {
    //     if (av[6] == std::string("-window"))
    //         robot.setFlag("window", true);
    //     else
    //         std::cout << "Unrecognized option: " << av[6] << std::endl;
    // }

//Robot robot(new Arena({123, 243}, {59, 34}), true); // subway surfer room
    robot.debug(std::stod(av[1]), std::stod(av[2]), std::stod(av[3]), std::stod(av[4]), std::stod(av[5]));
    std::cout << "Starting robot" << std::endl;
    robot.start();
    std::cout << "Robot started" << std::endl;
    robot.run();
    while(true);
    return 0;
}

// int timeOffset = 0;
// #include "./Lidar/Lidar.hpp"
// #include "./Display.hpp"
// #include "./Motors/Motors.hpp"
// #include "./pch.h"
// #define ever ;;

// int main(int ac, char **av) {
//     std::cout << ac << std::endl;
//     Arena arena({240, 363}, {120,39}); // arena
//     Display display(&arena);
//     Motors motors(arena);
//     Lidar lidar(arena, &display);

//     for(ever)
//     if (ac >= 3) {
//         motors.setSpeed(atoi(av[1]), atoi(av[2]));
//     }
//     return 0;
// }