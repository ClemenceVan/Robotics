#include "./Lidar.hpp"
#include "./Display.hpp"
#include "./Arena.hpp"
#include "./Motors.hpp"

// int main(void) {
//     Arena arena({27, 57}, {13.5, 10.5});

//     Lidar lidar(arena, true);
//     while (!lidar.isDataReady());

//     while (true) {
//         lidar.cox_linefit();
//         sleep(1);
//     }
// }

int main() {
    motors();
}