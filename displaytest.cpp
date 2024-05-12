#include <iostream>
#include <ncurses.h>
#include <iostream>
#include <sstream>

struct cout_redirect {
    cout_redirect( std::streambuf * new_buffer ) 
        : old( std::cout.rdbuf( new_buffer ) )
    { }

    ~cout_redirect( ) {
        std::cout.rdbuf( old );
    }

private:
    std::streambuf * old;
};

// Function to draw a square of a defined size
void draw_square(int width, int height) {
    int i, j;
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            if (i == 0 || i == height - 1)
                mvprintw(i, j*2, "-");  // Print horizontal line for top and bottom border
            else if (j == 0 || j == width - 1)
                mvprintw(i, j*2, "|");  // Print vertical line for left and right border
            else
                mvprintw(i, j*2, " "); // Print empty space for inside of the square
        }
    }
}

// Function to print points from an array of coordinates
void print_points(int coordinates[][2], int num_points) {
    int i;
    for (i = 0; i < num_points; i++) {
        mvprintw(coordinates[i][0], coordinates[i][1]*2, "x "); // Adjusting for the double width of the lines
    }
}
   
std::stringstream buffer;
std::streambuf * old = std::cout.rdbuf(buffer.rdbuf());

void print_buffer(const std::string& buf) {
    int max_x, max_y;
    getmaxyx(stdscr, max_y, max_x);

    int start_col = max_x / 2; // Start column for printing buffer
    std::string curLine;
    for (int line = 0; std::getline(buffer, curLine); line++) {
        mvprintw(line, start_col, curLine.c_str());
    }
}

int main() {
    int width = 10; // Width of the square
    int height = 8; // Height of the square
    int coordinates[][2] = {{2, 2}, {4, 5}, {7, 3}}; // Example coordinates
    int num_points = sizeof(coordinates) / sizeof(coordinates[0]);

    initscr(); // Initialize ncurses
    curs_set(0); // Hide the cursor
    noecho(); // Don't echo input
    keypad(stdscr, TRUE); // Enable special key input

    draw_square(width, height); // Draw the square
    print_points(coordinates, num_points); // Print the points

    int max_x, max_y;
    getmaxyx(stdscr, max_y, max_x);

    std::cout << "Bla" << std::endl;
    std::cout << "Bla2" << std::endl;
    std::cout << "Bla3" << std::endl;

    while (true) {
        print_buffer(buffer.str());
        refresh(); // Refresh the screen
    }
    
    refresh(); // Refresh the screen
    getch(); // Wait for a key press
    endwin(); // End ncurses

    return 0;
}
