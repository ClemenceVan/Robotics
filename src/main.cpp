#include "./Lidar/Lidar.hpp"
#include "./Display.hpp"
#include "./Arena.hpp"
// #include "./Motors.hpp"

#ifdef _WIN32
    #define PATH "../testfile90.txt"
#else
    #define PATH ""
#endif

int main(void) {
    // Arena arena({39, 54}, {39/2, 54/2});
    Arena arena({57, 27}, {14, 29});

    Lidar lidar(arena, false, PATH);
    while (!lidar.isDataReady());

    while (true) {
        lidar.cox_linefit();
        sleep(1);
    }
}

// int main() {
//     motors();
// }