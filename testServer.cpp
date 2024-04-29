#include <iostream>
#include "./libraries/asio/include/asio.hpp"

using asio::ip::tcp;

void handle_client(std::shared_ptr<tcp::socket> socket) {
    try {
        asio::streambuf buffer;
        std::size_t n;

        while (asio::read_until(*socket, buffer, '\n')) {
            std::istream input_stream(&buffer);
            std::string line;
            std::getline(input_stream, line);
            std::cout << "Received: " << line << std::endl;
        }
    } catch (std::exception& e) {
        std::cerr << "Exception in handle_client: " << e.what() << std::endl;
    }
}

int main() {
    try {
        asio::io_context io_context;
        tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), 1234));

        while (true) {
            std::shared_ptr<tcp::socket> socket = std::make_shared<tcp::socket>(io_context);
            acceptor.accept(*socket);
            std::cout << "New connection from: " << socket->remote_endpoint().address().to_string() << std::endl;
            std::thread(handle_client, std::move(socket)).detach();
        }
    } catch (std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
    }

    return 0;
}
