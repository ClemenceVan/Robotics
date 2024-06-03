#include <wiringPi.h>
#include "servo.hpp"

/*--------------------------------------- Do not change the pins --------------------------*/
const int PWM0_pin = 26; // GPIO 26 as WiringPi, <=> pin 32 (RPI ver 3B)
const int PWM1_pin = 23;  // GPIO 23 as WiringPi, <=> pin 33 (RPI ver 3B)
/*-----------------------------------------------------------------------------------------*/

static int INIT_RPI = 0;

void Init_RPI(void){

	INIT_RPI = 1;
	wiringPiSetup();
	pinMode(PWM0_pin, PWM_OUTPUT); /* set PWM0 pin as output */
	pwmSetMode(PWM_MODE_MS);
	pinMode(PWM1_pin, PWM_OUTPUT); /* set PWM1 pin as output */
	pwmSetMode(PWM_MODE_MS);
	pwmSetClock(384);  // 19.2 Mhz divided by 384.
	pwmSetRange(1000); // 19.2 Mhz divided by 384 divided by 1000 = 50Hz
}

void Servo(int Ch, int Deg) {

	if (!INIT_RPI){
		Init_RPI();
	}
	if (Ch == 0 && Deg > -1 && Deg < 181){ // range: one to two ms pwm
		pwmWrite(PWM0_pin, (50*Deg)/180 + 50);    // range 0deg = 1ms to 180deg = 2ms
	}
	else if (Ch == 1 && Deg > -1 && Deg < 181){ // range: one to two ms pwm
		pwmWrite(PWM1_pin, (50*Deg)/180 + 50);    // range 0deg = 1ms to 180deg = 2ms
	}
}

void up(){
	int i = 25;
		while(1){
		
		i--;
		Servo(0,i);
		delay(10);
		if(i == 0)
			return;
		
	}
	
}

void down(){
	
	int i = 0;
		while(1){
		
		i++;
		Servo(0,i);
		delay(10);
		if(i == 25)
			return;
		
	}
	
}
