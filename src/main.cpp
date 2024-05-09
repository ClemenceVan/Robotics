// #include "./Lidar/Lidar.hpp"
// #include "./Motors/Motors.hpp"
// #include "./Display.hpp"
// #include "./Robot/Arena.hpp"

// #include "./Motors.hpp"

// int main(void) {
//     // Arena arena({39, 54}, {39/2, 54/2});
//     Arena arena({57, 27}, {14, 29});

//     Lidar lidar(arena, false, PATH);
//     Motors motors(arena);

//     /* Lidar thread */
//     std::thread lidar_thread([&]() {
//         while (!lidar.isDataReady());
//         while (true) {
//             lidar.cox_linefit();
//             sleep(1);
//         }
//     });
//     lidar_thread.detach();
//     /* *** */

//     /* Motors thread */
//     while (true) {
//         motors.odometry();
//         sleep(1);
//     }
// }

#include "Robot/Robot.hpp"
#include "Robot/Arena.hpp"

int main(void) {
    Robot robot(new Arena({57, 27}, {14, 10}));
    robot.start();
    robot.run();
    while (true) {
        sleep(1);
    }
    return 0;
}