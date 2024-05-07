#include "./libraries/spi_com.h"
#include "./libraries/current/ina219.h"
#include "./Eigen/Dense"
#include <wiringPi.h>
#include <stdio.h>
#include <wiringPiSPI.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
// #include <ncurses.h>
#include <thread>

#include <ftxui/screen/screen.hpp>
#include <ftxui/screen/string.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/component/screen_interactive.hpp>


//g++ main.cpp -lpthread -lwiringPi -lrobotic_gcc /home/pi/Documents/Labs/Lab\ 3/libraries/ina219.so -o main

const float SHUNT_OHMS = 0.01;
const float MAX_EXPECTED_AMPS = 3.2;
const float Delay_ms = 1000.0;

MotorDataType MotorData;

static const int SPI_Channel = 1;

short Des_Speed_L = 0;
short Des_Speed_R = 0;
int Select = 0;
int Counter = 0;

//----odometry variables: -------
double x_pos = 0;
double y_pos = 0;
double a_pos = 0;//90*M_PI/180;
double x_pos_prev = 0;
double y_pos_prev = 0;
double a_pos_prev = 0;//90*M_PI/180;
double Dr = 0;
double Dl = 0;
double prev_enc_l = 0;
double prev_enc_r = 0;
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void tommy_odometry(double current_enc_r, double current_enc_l)
{

    double wheel_base = 70; // mm
    double wheel_diameter = 30; // mm
    double PULSES_PER_REVOLUTION = 1024; // ticks
    double circumference_wheel = wheel_diameter * M_PI;
    double MM_PER_PULSE = circumference_wheel / PULSES_PER_REVOLUTION;
    double gearbox_ratio = 10;
    //current_enc_l *= MM_PER_PULSE;
    //current_enc_r *= MM_PER_PULSE;

    //----Kinematic model of a differential drive robot: ------
    double C = PULSES_PER_REVOLUTION * 6.6 * gearbox_ratio * 4 / circumference_wheel; // C = Nbr of pulses per mm = pulses per revolution * gearbox ratio * 4 (counts per puls) /circumference wheel
    double dt = 0.05;//50 / 1000; // sample time
    double vl = (current_enc_l - prev_enc_l) / (dt*C);
    double vr = (current_enc_r - prev_enc_r) / (dt*C);

    double v = (vr+vl)/2;
    double w = (vr-vl)/wheel_base;
    double L = v*dt;
    //----Movement in robot coordinate system:-----
    double dA = w*dt; // A = theta = angle
    double dX =  L * cos(dA/2);
    double dY = L * sin(dA/2);
    //----create position in global coordinate system: -----
    x_pos = x_pos_prev + dX*cos(a_pos_prev) - dY*sin(a_pos_prev); 
    y_pos = y_pos_prev + dY*cos(a_pos_prev) - dX*sin(a_pos_prev); 
    a_pos = a_pos_prev + dA;
    
    // std::cout << "Position: x = " << x_pos << ", y = " << y_pos << ", angle = " << a_pos << std::endl;
    

    //-----update values:-----
    x_pos_prev = x_pos;
    y_pos_prev = y_pos;
    a_pos_prev = a_pos;
    prev_enc_l = current_enc_l;
    prev_enc_r = current_enc_r;
    
}

INA219 Ina(SHUNT_OHMS, MAX_EXPECTED_AMPS);
double offset_m1 = 0;// = MotorData.Encoder_M1;
double offset_m2 = 0;// = MotorData.Encoder_M2;

void init() {
	wiringPiSetup(); 
	wiringPiSPISetup(SPI_Channel, 1000000);
    MotorDataType motor;
    Send_Read_Motor_Data(&MotorData);
    
    Ina.configure(RANGE_16V, GAIN_8_320MV, ADC_12BIT, ADC_12BIT);
    // printf("\u001b[35m\n");
	
    Send_Read_Motor_Data(&MotorData);
    offset_m1 = MotorData.Encoder_M1;
    offset_m2 = -MotorData.Encoder_M2;
    // std::cout << "offset_m1: " << offset_m1 << " offset_m2: " << offset_m2 << std::endl;
}

int increment = 50;
int running = 1;

void terminal_thread() {
    using namespace ftxui;

    auto screen = ScreenInteractive::Fullscreen();

    auto container = Container::Horizontal({
        Renderer::Center(
            text(L"Press arrow keys")
        )
    });

    screen.Loop(container, [&](Event event) -> bool {
        if (event == Event::ArrowLeft)
            screen.PostEvent(Event::Custom(1));
        else if (event == Event::ArrowRight)
            screen.PostEvent(Event::Custom(2));
        else if (event == Event::ArrowUp)
            screen.PostEvent(Event::Custom(3));
        else if (event == Event::ArrowDown)
            screen.PostEvent(Event::Custom(4));
        else if (event == Event::Custom && event.custom_code == 1)
            container->Add(text(L"Left"));
        else if (event == Event::Custom && event.custom_code == 2)
            container->Add(text(L"Right"));
        else if (event == Event::Custom && event.custom_code == 3)
            container->Add(text(L"Up"));
        else if (event == Event::Custom && event.custom_code == 4)
            container->Add(text(L"Down"));

        if (event == Event::Custom)
            screen.PostEvent(Event::Custom(0));
        return event != Event::Custom(0);
    });

    return 0;
    // while(running) {
    //     printw("Use arrow keys to control the robot, q to exit\n");
	// 	printw("Speed_M1=%d Speed_M2=%d\n", Des_Speed_L, Des_Speed_R);
    //     refresh();

    //     int c = getch();
    //     switch(c) {
    //         case KEY_UP:
    //             Des_Speed_L += increment;
    //             Des_Speed_R += increment;
    //             break;
    //         case KEY_DOWN:
    //             Des_Speed_L -= increment;
    //             Des_Speed_R -= increment;
    //             break;
    //         case KEY_LEFT:
    //             Des_Speed_L -= increment;
    //             Des_Speed_R += increment;
    //             break;
    //         case KEY_RIGHT:
    //             Des_Speed_L += increment;
    //             Des_Speed_R -= increment;
    //             break;
    //         case ' ':
    //             Des_Speed_L = 0;
    //             Des_Speed_R = 0;
    //             break;
    //         case 'q':
    //             running = 0;
    //             break;
    //     }
	// 	// Send the desired speed to the motor controller
	// 	MotorData.Set_Speed_M1 = Des_Speed_L;
	// 	MotorData.Set_Speed_M2 = -Des_Speed_R;
	// 	// if controls reversed, comment out the above two lines and uncomment the following two lines
	// 	// MotorData.Set_Speed_M1 = Des_Speed_R;
	// 	// MotorData.Set_Speed_M2 = Des_Speed_L;
	// 	Send_Read_Motor_Data(&MotorData);
	// 	clear();
    // }
    // endwin();
}

int motors(void) {
    init();

    
    // std::string input;
    // std::cout << "Select initial speed: ";
    // std::cin >> input;
    // Des_Speed_L = std::stoi(input);
    // Des_Speed_R = std::stoi(input);

    std::thread terminal = std::thread(terminal_thread);
    terminal.detach();

    initscr();
    cbreak(); // Disable line buffering
    noecho(); // Don't echo keypresses to the screen
	keypad(stdscr, TRUE); // Enable arrow keys

	while(1) {
        float current = Ina.current();
        float Volt = Ina.supply_voltage();
        MotorData.Set_Speed_M1=Des_Speed_L; // m1 = leftmotor which is connected furthest away from hdmi port
        MotorData.Set_Speed_M2=-Des_Speed_R;  
        Send_Read_Motor_Data(&MotorData);

		delay(50);
        if (Volt < 10.5) {
            printf("\u001b[5m\u001b[40m\x1B[31m\u001b[1m💀💀 ERROR: voltage below 10.5 💀💀\x1B[37m\u001b[0m\u001b[35m\n");
            break;
        } else if (Volt < 10.8)
            printf("\u001b[5m\u001b[40m\x1B[33m\u001b[1m👉👈 WARNING: voltage below 10.8 🥺 \x1B[37m\u001b[0m\u001b[35m");
        
        double new_m1 = MotorData.Encoder_M1 - offset_m1;
        double new_m2 = -(MotorData.Encoder_M2) - offset_m2;
    
        tommy_odometry(new_m1, new_m2);

	}
	
}