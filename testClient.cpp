// #ifdef _WIN32
//     #include <winsock2.h>
//     #include <ws2tcpip.h>
//     #pragma comment(lib, "ws2_32.lib")
// #else
//     #include <sys/socket.h>
//     #include <netinet/in.h>
//     #include <unistd.h>
// #endif

// #include <iostream>
// #include <fstream>
// #include <string>

// int main(int ac, char** av) {
//     if (ac != 2) {
//         std::cout << "Usage: " << av[0] << " <test_data_file>" << std::endl;
//         return 1;
//     }
//     std::cout << "Opening test data file ..." << std::endl;
//     std::ifstream data(av[1]);
//     if (!data.is_open()) {
//         std::cout << "Error: could not open file " << av[1] << std::endl;
//         return 1;
//     }
    
//     #ifdef _WIN32
//         WSADATA wsaData;
//         WSAStartup(MAKEWORD(2, 2), &wsaData);
//     #endif

//     int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
//     if (clientSocket == -1) {
//         std::cout << "Socket creation failed...\n";
//         exit(0);
//     }
//     std::cout << "Socket successfully created..\n";

//     struct sockaddr_in serverAddress;
//     serverAddress.sin_family = AF_INET;
//     serverAddress.sin_port = htons(9888);
//     serverAddress.sin_addr.s_addr = inet_addr("127.0.0.1");

//     if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) != 0) {
//         std::cout << "Connection with the server failed...\n";
//         exit(0);
//     }

//     std::cout << "Connected to the server..\n";
//     for (std::string line; std::getline(data, line);) {
//         send(clientSocket, line.c_str(), line.size(), 0);
//         std::cout << "Sent: " << line << std::endl;
//     }
// }


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

    for (std::string line; std::getline(data, line);) {
        asio::write(socket, asio::buffer(line));
        std::cout << "Sent: " << line << std::endl;
    }
    while (true);
}
