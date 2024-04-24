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

    int CanvasWidth = 100;
    int CanvasHeight = 100;

    int width = 27;
    int height = 57;

    double start_x = 13.5; // to be changed when start
    double start_y = 10.5; // to be changed when start

    auto c = Canvas(100, 100);

    int CenterPointX = CanvasWidth/2;
    int CenterPointY = CanvasHeight/2;


    Eigen::MatrixXi line_model(4,4);
    line_model <<  0, 0, 0, height,
                0, 0, width, 0,
                0, height, width, height,
                width, 0, width, height;
    
    int xx = CenterPointX - width/2;
    int yy = CenterPointY - height/2;
    
    for (int i = 0; i < line_model.rows(); i++)
        c.DrawPointLine(xx + line_model(i,0), yy + line_model(i,1), line_model(i, 2) + xx, line_model(i, 3) + yy, Color::Red);
    
    for (int i = 0; i < positions.rows(); i++)
        c.DrawPoint(CenterPointX - positions(i, 0) + width/2 - start_x, CenterPointY - positions(i, 1) + height/2 - start_y, true, Color::Blue);
    
    for (int i = 0; i < new_positions.rows(); i++)
        c.DrawPoint(CenterPointX - new_positions(i, 0) + width/2 - start_x, CenterPointY - new_positions(i, 1) + height/2 - start_y, true, Color::Green);

    auto document = canvas(&c) | border;

    auto screen = Screen::Create(Dimension::Fit(document));
    Render(screen, document);
    screen.Print();
    getchar();

    return 0;
}