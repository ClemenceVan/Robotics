#include "./Robot/Robot.hpp"
#include "./Robot/Arena.hpp"

int main(int ac, char **av) {
    // Robot robot(new Arena({39, 54}, {39/2, 54/2}), true); // big box
    Robot robot(new Arena({363, 240}, {178, 33}), true); // arena
    // Robot robot(new Arena({240, 363}, {178, 33}), true); // arena
    robot.start();
    robot.run();
    return 0;
}