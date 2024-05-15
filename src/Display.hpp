#pragma once
#include <stddef.h>    // for size_t
#include <array>       // for array
#include <atomic>      // for atomic
#include <chrono>      // for operator""s, chrono_literals
#include <cmath>       // for sin
#include <functional>  // for ref, reference_wrapper, function
#include <memory>      // for allocator, shared_ptr, __shared_ptr_access
#include <string>  // for string, basic_string, char_traits, operator+, to_string
#include <thread>   // for sleep_for, thread
#include <utility>  // for move
#include <vector>   // for vector

#include "ftxui/dom/canvas.hpp"  // for Canvas
#include "ftxui/dom/node.hpp"    // for Render
#include "ftxui/screen/color.hpp"  // for Color, Color::Red, Color::Blue, Color::Green, ftxui

#include "Eigen/Dense"

#include "./include.hpp"
#include "./Robot/Arena.hpp"

using namespace ftxui;

class Display {
    private :
        int CanvasWidth = 100;
        int CanvasHeight = 100;
        int scale = 4;
        double CenterPointX = CanvasWidth/4;
        double CenterPointY = CanvasHeight/4;
        double rectStartX = CenterPointX - width / 2;
        double rectStartY = CenterPointY + height / 2;

        int width;
        int height;
        double start_x;
        double start_y;
        double start_angle;
        Eigen::MatrixXd line_model = Eigen::MatrixXd(4, 4);

        ftxui::Canvas c;
        ftxui::Canvas c_r;
    public:
        Display(Arena *arena) {
            width = arena->getSize().first;
            height = arena->getSize().second;
            start_x = arena->getOrigin().first;
            start_y = arena->getOrigin().second;
            line_model = arena->getLineModel();
            c = ftxui::Canvas(CanvasWidth, CanvasHeight);
            c_r = ftxui::Canvas(CanvasWidth, CanvasHeight);
        }

        ~Display() {
            // Destructor
        }

        void coxDrawing(Eigen::MatrixXd positions, Eigen::MatrixXd new_positions) {
            c = ftxui::Canvas(CanvasWidth, CanvasHeight);
            // Adjust the coordinates of the line model based on the center point.
            for (int i = 0; i < line_model.rows(); i++)
                c.DrawPointLine((line_model(i, 0) + CenterPointX) / scale,
                                (line_model(i, 1) + CenterPointY) / scale,
                                (line_model(i, 2) + CenterPointX) / scale,
                                (line_model(i, 3) + CenterPointY) / scale, Color::Red);
                // Adjust the coordinates of positions and new_positions based on the center point.

            for (int i = 0; i < positions.rows(); i++)
                c.DrawPoint((positions(i, 0) + CenterPointX) / scale, (positions(i, 1) + CenterPointY) / scale, true, Color::Blue);

            for (int i = 0; i < new_positions.rows(); i++)
                c.DrawPoint((new_positions(i, 0) + CenterPointX) / scale, (new_positions(i, 1) + CenterPointY) / scale, true, Color::Green);
                auto document = canvas(&c) | border;
            this->draw();
        }

        void drawCoordinates(double cX, double cY, double cA, double oX, double oY, double oA, double kX, double kY, double kA) {
            std::cout << "Position: " << std::endl;
            std::cout << "Odometry: " << oX << " " << oY << " " << oA << std::endl;
            std::cout << "Cox: " << cX << " " << cY << " " << cA << std::endl;
            std::cout << "Kalman: " << kX << " " << kY << " " << kA * 180 / M_PI << std::endl;

            c.DrawPoint((kX + CenterPointX) / scale, (kY + CenterPointY) / scale, true, Color::Blue);
        }

        void draw() {
            auto element = ftxui::text(L"x") | ftxui::color(Color::Red);
            auto document = canvas(&c) | border;
            auto screen = Screen::Create(Dimension::Fit(document));
            Render(screen, document);
            screen.Print();
            std::cout << std::endl;
        }
};