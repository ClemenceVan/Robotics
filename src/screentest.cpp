// Copyright 2020 Arthur Sonzogni. All rights reserved.
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.
#include <stdio.h>                 // for getchar
#include <cmath>                   // for cos
#include <ftxui/dom/elements.hpp>  // for Fit, canvas, operator|, border, Element
#include <ftxui/screen/screen.hpp>  // for Pixel, Screen
#include <vector>                   // for vector, allocator
#include <iostream>

#include "ftxui/dom/canvas.hpp"  // for Canvas
#include "ftxui/dom/node.hpp"    // for Render
#include "ftxui/screen/color.hpp"  // for Color, Color::Red, Color::Blue, Color::Green, ftxui

#include "../Eigen/Dense"
/*
class Display {
    int CanvasWidth = 100;
    int CanvasHeight = 100;

    //// Move to include /////
    int width = 27;
    int height = 57;

    double start_x = 13.5; // to be changed when start
    double start_y = 10.5; // to be changed when start
    //////////////////

    ftxui::Canvas c;
    Display() {

        c = ftxui::Canvas(CanvasWidth, CanvasHeight);
        int CenterPointX = CanvasWidth/2;
        int CenterPointY = CanvasHeight/2;

        for (int i = 0; i < line_model.rows(); i++)
            c.DrawPointLine(xx + line_model(i,0), yy + line_model(i,1), line_model(i, 2) + xx, line_model(i, 3) + yy, Color::Red);
    }
};*/

int screen(Eigen::MatrixXd positions, Eigen::MatrixXd new_positions) {
    using namespace ftxui;

    int CanvasWidth = 150;
    int CanvasHeight = 100;
    int scale = 3;

    int width = 244 / scale;
    int height = 365 / scale;


    double start_x = 149; // to be changed when start
    double start_y = 87.5; // to be changed when start

    auto c = Canvas(150, 100);

    int CenterPointX = CanvasWidth/4;
    int CenterPointY = CanvasHeight/4;

    int rectStartX = CenterPointX - width / 2;
    int rectStartY = CenterPointY + height / 2;


    Eigen::MatrixXi line_model(4,4);
    line_model <<  0, 0, 0, height,
                0, 0, width, 0,
                0, height, width, height,
                width, 0, width, height;

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

    auto screen = Screen::Create(Dimension::Fit(document));
    Render(screen, document);
    screen.Print();
    // getchar();

    return 0;
}