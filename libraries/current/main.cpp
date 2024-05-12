/*compile with; g++ -o a.out ina219.so -lwiringPi main.cpp

*/

#include <iostream>
#include <stdio.h> 
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "ina219.h"

const float SHUNT_OHMS = 0.01;
const float MAX_EXPECTED_AMPS = 3.2;
const float Delay_ms = 1000.0;

int main(void){

    INA219 Ina(SHUNT_OHMS, MAX_EXPECTED_AMPS);
    Ina.configure(RANGE_16V, GAIN_8_320MV, ADC_12BIT, ADC_12BIT);
    printf("supply_voltage_V,      current_mA,\n");
    while(true){
        float current = Ina.current();
        float Volt = Ina.supply_voltage();
        printf("voltage=%3.1fV    current=%4.0fmA\n",Volt,current);
        usleep(Delay_ms * 1000);
    }
	return 0;
}
