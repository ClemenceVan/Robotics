#include "Robot/Robot.hpp"
#include "Robot/Arena.hpp"

int main(void) {
    Robot robot(new Arena({57, 27}, {14, 10}));
    robot.start();
    robot.run();
    return 0;
}