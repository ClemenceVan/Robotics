#include "./libraries/current/ina219.h"
#include <iostream>
#include <unistd.h>

int main() {
    const float SHUNT_OHMS = 0.01;
    const float MAX_EXPECTED_AMPS = 3.2;
    const float Delay_ms = 1000.0;
    INA219 Ina(SHUNT_OHMS, MAX_EXPECTED_AMPS);
    Ina.configure(RANGE_16V, GAIN_8_320MV, ADC_12BIT, ADC_12BIT);
    while(true) {
        float current = Ina.current();
        float Volt = Ina.supply_voltage();
        if (Volt < 10.5) {
            while(true) std::cout << "\u001b[5m\u001b[40m\x1B[31m\u001b[1m💀💀 ERROR: voltage below 10.5 💀💀\x1B[37m\u001b[0m\u001b[35m\n" << std::endl;
        } else if (Volt < 10.8)
            std::cout << "\u001b[5m\u001b[40m\x1B[33m\u001b[1m👉👈 WARNING: voltage below 10.8 🥺 \x1B[37m\u001b[0m\u001b[35m" << std::endl;
        sleep(5);
    }
}