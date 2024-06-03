#include "Lidar.hpp"

void writeDriver(std::string msg) {
    const char* message;

    if (msg == "start") {
        std::cout << "Starting driver" << std::endl;
    } else if (msg == "stop") {
        std::cout << "Stopping driver" << std::endl;
    } else {
        std::cerr << "Invalid message" << std::endl;
        return;
    }
    
    const char* TCP_IP = "127.0.0.1";
    const int TCP_PORT = 9887;
    const int BUFFER_SIZE = 1024;

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "Error creating socket" << std::endl;
        throw std::runtime_error("Could not create socket");
    }

    // Server address structure
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(TCP_PORT);
    inet_pton(AF_INET, TCP_IP, &server_addr.sin_addr);

    // Connect to the server
    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Error connecting to server" << std::endl;
        close(sock);
        throw std::runtime_error("Could not connect to server");
    }

    // Send data to the server
    if (send(sock, message, strlen(message), 0) < 0) {
        std::cerr << "Error sending data" << std::endl;
        close(sock);
        return;
    }
}

Lidar::Lidar(Arena arena, Display *disp, std::string path): cox("cox.txt") {
    this->arena = &arena;
    this->line_model = arena.getLineModel();
    this->posX = std::get<0>(arena.getOrigin());
    this->posY = std::get<1>(arena.getOrigin());
    this->RMatrix <<  cos(this->_gamma), -sin(this->_gamma), this->alpha,
                    sin(this->_gamma), cos(this->_gamma), this->beta,
                    0, 0, 1;
    // robot to world coordinates
    this->CMatrix <<  cos(this->posA), -sin(this->posA), this->posX,
                sin(this->posA), cos(this->posA), this->posY,
                0, 0, 1;
    this->display = disp;
    if (path == "") {
        writeDriver("start");
        this->serverAddress.sin_family = AF_INET;
        this->serverAddress.sin_port = htons(PORT);
        this->serverAddress.sin_addr.s_addr = INADDR_ANY;
        bind(this->serverSocket, (struct sockaddr *)&this->serverAddress, sizeof(this->serverAddress));
        listen(this->serverSocket, 5);
        newSock = accept(this->serverSocket, NULL, NULL);

        // std::cout << "New connection from: " << newSock << std::endl;
        pollingThread = std::thread(&Lidar::pollLidarData, this);
        pollingThread.detach();
    } else {
        data.open(path);
        if (!data.is_open()) throw std::runtime_error("Could not open file");
        pollingThread = std::thread(&Lidar::readFileData, this);
        pollingThread.detach();
    }
}

Lidar::~Lidar() {
    writeDriver("stop");
    if (_isPolling) {
        close(this->newSock);
        close(this->serverSocket);
        // this->socket.close();
    }
    cox.close(); // close txt file
}

void Lidar::readFileData() {
    int certainty, angle, distance;
    while (true) {
        while (this->data >> certainty >> angle >> distance) {
            // // std::cout << "certainty: " << certainty << " angle: " << angle << " distance: " << distance << std::endl;
            if (certainty < 10 || distance == 0) {std::cout << "Skipped data from lidar" << std::endl; continue;}
            float angles_rad = fmod((angle *(M_PI / 180)),2*M_PI);
            std::cout << "certainty: " << certainty << " angle: " << angle << " distance: " << distance << std::endl;
            mtx.lock();   
            if (lines == 500)
                positionMatrixPoll.block(0, 0, positionMatrixPoll.rows() - 1, 3) = positionMatrixPoll.block(1, 0, positionMatrixPoll.rows() - 1, 3);
            positionMatrixPoll(lines, 0) = distance * cos(-angles_rad) / 10;
            positionMatrixPoll(lines, 1) = distance * sin(-angles_rad) / 10;
            positionMatrixPoll(lines, 2) = 1;
            if (lines < 500) lines++;
            mtx.unlock();
        }
        this->data.clear();
        this->data.seekg(0, std::ios::beg);
    }
}

void Lidar::pollLidarData() {
    while(true) {
        // Receive header
        valread = read(newSock, buffer2, 5);
        if (valread != 5) {
            std::cout << "Could not read header" << std::endl;
            for (int i = 0; i < 5; i++) {
                std::cout << (int)buffer2[i] << " ";
            }
            throw std::runtime_error("Could not read header : " + std::to_string(valread));   
            // std::cout << "Could not read header" << std::endl;
            // continue;
        }

        // Parse header
        if (buffer2[0] == (char)0xA5) {
            unsigned int data_length = ((buffer2[2] << 16) | (buffer2[3] << 8) | buffer2[4]) & 0xFFFFFF;

            // Receive data
            valread = read(newSock, buffer2, data_length);
            if (valread != data_length) break;

            unsigned int angle = (((unsigned int)buffer2[1] >> 1) + (((unsigned int)buffer2[2]) << 7)) >> 6;
            unsigned int distance = (((unsigned int)buffer2[3]) + (((unsigned int)buffer2[4]) << 8)) >> 2;
            unsigned char quality = buffer2[0] >> 2;
            // std::cout << "quality: " << (int)quality << " angle: " << angle << " distance: " << distance << std::endl;
            if (quality < 10 || distance == 0) continue;
            double angles_rad = fmod((angle *(M_PI / 180)),2*M_PI);
            mtx.lock();
            if (lines == 499)
                positionMatrixPoll.block(0, 0, positionMatrixPoll.rows() - 1, 3) = positionMatrixPoll.block(1, 0, positionMatrixPoll.rows() - 1, 3);
            positionMatrixPoll(lines, 0) = distance * cos(-angles_rad) / 10;
            positionMatrixPoll(lines, 1) = distance * sin(-angles_rad) / 10;
            positionMatrixPoll(lines, 2) = 1;
            if (lines < 499) lines++;
            // std :: cout << "lines: " << lines << std::endl;
            mtx.unlock();
        } else {
            std::cout << "Invalid header" << std::endl;
            for (int i = 0; i < 5; i++) {
                std::cout << (int)buffer2[i] << " ";
            }
            throw std::runtime_error("Invalid header : " + std::to_string(buffer2[0]));
        }
    }
}


int Lidar::isDataReady() {
    mtx.lock();
    int l = this->lines;
    mtx.unlock();
    if (l >= 499) return 1;
    return 0;
}

Eigen::MatrixXd Lidar::getData() {
    mtx.lock();
    Eigen::MatrixXd m = this->positionMatrixPoll;
    this->positionMatrixPoll = Eigen::MatrixXd::Zero(500, 3);
    this->lines = 0;
    mtx.unlock();
    return m;
}
    
double Lidar::getPosX() {
    return posX;
}

double Lidar::getPosY() {
    return posY;
}

double Lidar::getPosA() {
    return posA;
}

Eigen::MatrixXd Lidar::getCovariance() {
    return covariance;
}

void Lidar::waitForUpdate() {
    while (!_positionsUpdated);
    _positionsUpdated = 0;
}

void Lidar::updatePosition(double x, double y, double a) {
    this->positionMutex.lock();
    this->posX = x;
    this->posY = y;
    this->posA = a;
    this->positionMutex.unlock();
    _positionsUpdated = 1;
}