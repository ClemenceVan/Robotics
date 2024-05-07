#include <unistd.h>
#include "asio.hpp"
#include <iostream>
#include <fstream>


int main(int ac, char** av) {
    if (ac != 2) {
        std::cout << "Usage: " << av[0] << " <test_data_file>" << std::endl;
        return 1;
    }
    std::cout << "Opening test data file ..." << std::endl;
    std::ifstream data(av[1]);
    if (!data.is_open()) {
        std::cout << "Error: could not open file " << av[1] << std::endl;
        return 1;
    }

    while (true) {
        try {
            #ifdef _WIN32
                WSADATA wsaData;
                WSAStartup(MAKEWORD(2, 2), &wsaData);
            #endif
            std::cout << "Connecting to server ..." << std::endl;
            asio::io_context io_context;
            asio::ip::tcp::resolver resolver(io_context);
            asio::ip::tcp::resolver::results_type endpoints = resolver.resolve("localhost", "9888");
            asio::ip::tcp::socket socket(io_context);
            asio::connect(socket, endpoints);

            while(true) {
                for (std::string line; std::getline(data, line);) {
                    asio::write(socket, asio::buffer(line));
                    std::cout << "Sent: " << line << std::endl;
                    // sleep(1);
                }
                data.clear();
                data.seekg(0, std::ios::beg);        
            }
        } catch (std::exception& e) {
            std::cerr << e.what() << std::endl;
        }
    }
}
