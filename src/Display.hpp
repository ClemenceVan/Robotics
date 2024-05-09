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

// #include "../dom/color_info_sorted_2d.ipp"  // for ColorInfoSorted2D
#include "ftxui/component/component.hpp"  // for Checkbox, Renderer, Horizontal, Vertical, Input, Menu, Radiobox, ResizableSplitLeft, Tab
#include "ftxui/component/component_base.hpp"  // for ComponentBase, Component
#include "ftxui/component/component_options.hpp"  // for MenuOption, InputOption
#include "ftxui/component/event.hpp"              // for Event, Event::Custom
#include "ftxui/component/screen_interactive.hpp"  // for Component, ScreenInteractive
#include "ftxui/dom/elements.hpp"  // for text, color, operator|, bgcolor, filler, Element, vbox, size, hbox, separator, flex, window, graph, EQUAL, paragraph, WIDTH, hcenter, Elements, bold, vscroll_indicator, HEIGHT, flexbox, hflow, border, frame, flex_grow, gauge, paragraphAlignCenter, paragraphAlignJustify, paragraphAlignLeft, paragraphAlignRight, dim, spinner, LESS_THAN, center, yframe, GREATER_THAN
#include "ftxui/dom/flexbox_config.hpp"  // for FlexboxConfig
#include "ftxui/screen/color.hpp"  // for Color, Color::BlueLight, Color::RedLight, Color::Black, Color::Blue, Color::Cyan, Color::CyanLight, Color::GrayDark, Color::GrayLight, Color::Green, Color::GreenLight, Color::Magenta, Color::MagentaLight, Color::Red, Color::White, Color::Yellow, Color::YellowLight, Color::Default, Color::Palette256, ftxui
#include "ftxui/screen/color_info.hpp"  // for ColorInfo
#include "ftxui/screen/terminal.hpp"    // for Size, Dimensions

#include "Eigen/Dense"

#include "./include.hpp"
#include "../Robot/Arena.hpp"

using namespace ftxui;

class Display {
    private :
    int CanvasWidth;
    int CanvasHeight;
    int ArenaWidth;
    int ArenaHeight;
    int OriginX;
    int OriginY;
    int CenterPointX;
    int CenterPointY;
    int scale = 1;
    Eigen::MatrixXd line_model;
    // ftxui::Screen screen = ftxui::Screen::Create(ftxui::Dimension::Full());
    ftxui::Screen screen = ftxui::Screen(100, 100);
    ftxui::Canvas c;
    ftxui::Element document;

    Eigen::MatrixXd positions;
    Eigen::MatrixXd new_positions;

    std::thread disLoop;
    std::thread refresh_ui;

    std::atomic<bool> refresh_ui_continue;

    ftxui::Component lidar = Renderer([this] {
        auto c = ftxui::Canvas(50, 75);
        // c.DrawBlockLine(0, 0, 100, 100, ftxui::Color::White);
        // c.DrawPointLine(0, 0, 50, 50, ftxui::Color::Red);
        // for (int i = 0; i < line_model.rows(); i++)
        //     c.DrawPointLine(xx + line_model(i,0), yy + line_model(i,1), line_model(i, 2) + xx, line_model(i, 3) + yy, ftxui::Color::Red);
        
        for (int i = 0; i < line_model.rows(); i++) {
            c.DrawPointLine((line_model(i, 0) + 5) / scale,
                            (line_model(i, 1) + 5) / scale,
                            (line_model(i, 2) + 5) / scale,
                            (line_model(i, 3) + 5) / scale, Color::Red);
            
        }

        for (int i = 0; i < positions.rows(); i++)
            c.DrawPoint((positions(i, 0) + CenterPointX) / scale, (positions(i, 1) + CenterPointY) / scale, true, Color::Blue);

        for (int i = 0; i < new_positions.rows(); i++)
            c.DrawPoint((new_positions(i, 0) + CenterPointX) / scale, (new_positions(i, 1) + CenterPointY) / scale, true, Color::Green);

        std::cout << positions << std::endl;
        // return canvas(std::move(c));
        return vbox({
            window(text("Position:"), canvas(std::move(c)) | border | yflex),
            //canvas(std::move(c)) | border)
        }) | yframe | flex;

    });

    ftxui::Component camera = Renderer([this] {
        std::string str =
            "Lorem Ipsum is simply dummy text of the printing and typesetting "
            "industry. Lorem Ipsum has been the industry's standard dummy text "
            "ever since the 1500s, when an unknown printer took a galley of type "
            "and scrambled it to make a type specimen book.";
        return vbox({
            window(text("Camera data:"), paragraphAlignLeft(str))
        }) | yframe | flex;
    });
    
    ftxui::Component settings = Renderer([this] {
        std::string str =
            "Lorem Ipsum is simply dummy text of the printing and typesetting "
            "industry. Lorem Ipsum has been the industry's standard dummy text "
            "ever since the 1500s, when an unknown printer took a galley of type "
            "and scrambled it to make a type specimen book.";
        return vbox({
            window(text("Setting:"), paragraphAlignLeft(str))
        }) | yframe | flex;
    });

    public:
    Display(std::pair<int, int> canvasSize, Arena arena) {
        
        this->CanvasWidth = std::get<0>(canvasSize);
        this->CanvasHeight = std::get<1>(canvasSize);
        this->ArenaWidth = std::get<0>(arena.getSize());
        this->ArenaHeight = std::get<1>(arena.getSize());
        this->OriginX = std::get<0>(arena.getOrigin());
        this->OriginY = std::get<1>(arena.getOrigin());
        this->CenterPointX = CanvasWidth/2;
        this->CenterPointY = CanvasHeight/2;
        line_model = arena.getLineModel();
    
        c = ftxui::Canvas(CanvasWidth, CanvasHeight);

        int xx = CenterPointX - ArenaWidth/2;
        int yy = CenterPointY - ArenaHeight/2;
        auto screen = ScreenInteractive::Fullscreen();
        int shift = 0;
        // line_model <<
        //     0, 0, 0, 27,
        //     0, 27, 57, 27,
        //     57, 27, 57, 0,
        //     57, 0, 0, 0;
        
        
        


        int tab_index = 0;
        std::vector<std::string> tab_entries = {
            "Lidar", "Camera", "Settings"
        };
        auto tab_selection =
            Menu(&tab_entries, &tab_index, MenuOption::HorizontalAnimated());
        auto tab_content = Container::Tab(
            {
                lidar,
                camera,
                settings
            },
            &tab_index);

        auto exit_button = Button(
            "Exit", [&] { screen.Exit(); }, ButtonOption::Animated());

        auto main_container = Container::Vertical({
            Container::Horizontal({
                tab_selection,
                exit_button,
            }),
            tab_content,
        });

        auto main_renderer = Renderer(main_container, [&] {
            return vbox({
                text("THOMAS the TRAIN") | bold | hcenter,
                hbox({
                    tab_selection->Render() | flex,
                    exit_button->Render(),
                }),
                tab_content->Render() | flex,
            });
        });

        refresh_ui_continue = true;
        refresh_ui = std::thread([&] {
            while (refresh_ui_continue) {
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(0.05s);
            // The |shift| variable belong to the main thread. `screen.Post(task)`
            // will execute the update on the thread where |screen| lives (e.g. the
            // main thread). Using `screen.Post(task)` is threadsafe.
            screen.Post([&] { shift++; });
            screen.Post([&] { lidar = lidar; });
            // After updating the state, request a new frame to be drawn. This is done
            // by simulating a new "custom" event to be handled.
            screen.Post(Event::Custom);
            }
        });

        screen.Loop(main_renderer);
        refresh_ui_continue = false;
        refresh_ui.join();
        disLoop.detach();
    }

    ~Display() {
        refresh_ui_continue = false;
        refresh_ui.join();
        disLoop.join();
    }

    void updatePositions(Eigen::MatrixXd positions, Eigen::MatrixXd new_positions) {
        std::cout << "Updating positions" << std::endl;
        this->positions = positions;
        this->new_positions = new_positions;
    }

    void draw() {
        // ftxui::Render(screen, document);
        // screen.Print();
    }
};