#pragma once
#include "ftxui/dom/canvas.hpp"  // for Canvas
#include "ftxui/dom/node.hpp"    // for Render
#include "ftxui/screen/color.hpp"  // for Color, Color::Red, Color::Blue, Color::Green, ftxui

#include "./include.hpp"


class Display {
    private :
    int CanvasWidth = 100;
    int CanvasHeight = 100;
    int CenterPointX = CanvasWidth/2;
    int CenterPointY = CanvasHeight/2;
    // ftxui::Screen screen = ftxui::Screen::Create(ftxui::Dimension::Full());
    ftxui::Screen screen = ftxui::Screen(100, 100);
    ftxui::Canvas c;
    ftxui::Element document;

    public:
    Display(int canvaWidth, int canvaHeight, Eigen::MatrixXd line_model): CanvasWidth(canvaWidth), CanvasHeight(canvaHeight) {
        c = ftxui::Canvas(CanvasWidth, CanvasHeight);
        CenterPointX = CanvasWidth/2;
        CenterPointY = CanvasHeight/2;

        int xx = CenterPointX - width/2;
        int yy = CenterPointY - height/2;
        document = ftxui::canvas(&c) | ftxui::border;
        screen = ftxui::Screen::Create(ftxui::Dimension::Fit(document));
        for (int i = 0; i < line_model.rows(); i++)
            c.DrawPointLine(xx + line_model(i,0), yy + line_model(i,1), line_model(i, 2) + xx, line_model(i, 3) + yy, ftxui::Color::Red);
    }

    void draw() {
        ftxui::Render(screen, document);
        screen.Print();
    }

    void addFigure(Eigen::MatrixXd positions, ftxui::Color color) {
        for (int i = 0; i < positions.rows(); i++)
            c.DrawPoint(CenterPointX - positions(i, 0) + width/2 - (int)start_x, CenterPointY - positions(i, 1) + height/2 - (int)start_y, true, color);
    }
};