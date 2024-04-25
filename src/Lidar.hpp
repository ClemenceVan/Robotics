#include "./include.hpp"
#include "./screentest.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sys/types.h>
#include "asio/include/asio.hpp"

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <unistd.h>
#endif

class Lidar {
    private:
    int width;
    int height;
    int alpha, beta = 0;
    int _gamma = (-90 * (int)M_PI / 180) / 4;
    double start_x = 13.5; // to be changed when start
    double start_y = 10.5; // to be changed when start
    int start_angle = 0;
    double ddx = 0;
    double ddy = 0;
    double dda = 0;
    Eigen::MatrixXd RMatrix = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd CMatrix = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd positionMatrix = Eigen::MatrixXd::Zero(3, 1);
    Eigen::Matrix<double, 4, 4> line_model;

    std::ifstream data;

    Display *display;

    bool _isPooling = false;
    char buffer[1024] = {0};
    ssize_t valread;
    SOCKET serverSocket;
    SOCKET newSock;

    
    asio::io_service ioService;
    asio::ip::tcp::acceptor acceptor;
    asio::ip::tcp::socket socket;

    public:
    Lidar(int width = 27, int height = 57, bool display = true, std::string path = ""):
        acceptor(this->ioService, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), PORT)), socket(this->ioService),
        width(width), height(height) {
        this->RMatrix << cos(alpha), -sin(alpha), 0,
                    sin(alpha), cos(alpha), 0,
                    0, 0, 1;
                
        this->CMatrix << cos(beta), 0, sin(beta),
                    0, 1, 0,
                    -sin(beta), 0, cos(beta);

        this->line_model <<  0, 0, 0, height,
                    0, 0, width, 0,
                    0, height, width, height,
                    width, 0, width, height;

        if (display) this->display = new Display(60, 60, line_model);
        if (path != "") {
            data.open(path);
            if (!data.is_open()) throw std::runtime_error("Could not open file");
        } else {
            // serverSocket = socket(AF_INET, SOCK_STREAM, 0);
            // sockaddr_in serverAddress;
            // serverAddress.sin_family = AF_INET;
            // serverAddress.sin_port = htons(PORT);
            // serverAddress.sin_addr.s_addr = INADDR_ANY;
            // bind(serverSocket, (sockaddr *)&serverAddress, sizeof(serverAddress));
            // listen(serverSocket, 5);
            // std::cout << "Listening on port " << PORT << std::endl;
            // newSock = accept(serverSocket, NULL, NULL);
            // std::cout << "Connected to client " << newSock << std::endl;
            // _isPooling = true;
            
    

            // acceptor.async_accept(socket, [this](std::error_code ec) {
            // if (!ec) {
            //     std::cout << "Connected to client" << std::endl;
            //     _isPooling = true;
            // } else {
            //     std::cerr << "Accept error: " << ec.message() << std::endl;
            // }});
            acceptor.accept(socket);
            std::cout << "Connected to client" << std::endl;
            _isPooling = true;
                // socket.async_read_some(asio::buffer(buffer, sizeof(buffer)),
                //                 [this](std::error_code ec, std::size_t length) {
                // if (!ec) {
                //     // Process received data here
                //     std::cout << "Received data: " << std::string(buffer, length) << std::endl;
                // } else {
                //     std::cerr << "Read error: " << ec.message() << std::endl;
                //     _isPooling = false;
                //     socket.close();
                // }});
        }
    }

    ~Lidar() {
        if (data.is_open()) data.close();
        if (_isPooling) {
            close(newSock);
            close(serverSocket);
        }
    }

    void poolLidarData() {
        
        // socket.read_some(asio::buffer(buffer, sizeof(buffer)));
        // std::cout << "Received data: " << std::string(buffer) << std::endl;
        std::array<char, 128> buffer;
        asio::error_code error;
        asio::streambuf response;
        // size_t length = socket.read_some(asio::buffer(buffer), error);
        size_t length = asio::read_until(socket, response, '\n', error);
        if (error == asio::error::eof) {
            std::cout << "Connection closed by peer" << std::endl;
        } else if (error) {
            throw asio::system_error(error);
        }
        // std::cout << "Received data: " << buffer.data() << std::endl;
        std::cout << "Received data: " << asio::buffer_cast<const char*>(response.data()) << std::endl;
        // read_until(asio::buffer(buffer, sizeof(buffer)), '\n');

        // if (!_isPooling) return;
        // valread = recv(newSock, buffer, 5, 0);
        // if (valread != 5) return;

        // // Parse header
        // if (buffer[0] == (char)0xA5) {
        //     unsigned int data_length = ((buffer[2] << 16) | (buffer[3] << 8) | buffer[4]) & 0xFFFFFF;

        //     // Receive data
        //     valread = recv(newSock, buffer, data_length, 0);
        //     if (valread != data_length) return;

        //     unsigned int angle = (((unsigned int)buffer[1] >> 1) + (((unsigned int)buffer[2]) << 7)) >> 6;
        //     unsigned int distance = (((unsigned int)buffer[3]) + (((unsigned int)buffer[4]) << 8)) >> 2;
        //     unsigned char quality = buffer[0] >> 2;

        //     std::cout << static_cast<int>(quality) << " " << angle << " " << distance << std::endl;

        //     int angles_rad = M_PI * angle / 180;

        //     // float x = distance * cos(angles_rad);
        //     // float y = distance * sin(angles_rad);
        //     //std::cout << static_cast<int>(quality) << " " << angle << " " << distance << " " << x << " " << y << std::endl;
            
        //     // m(0, 0) = cos();
        //     // m(0, 1) = -sin();
        //     // m(0, 2) = x;//S alpha
        //     // m(1, 0) = sin();//_gamma
        //     // m(1, 1) = cos();//_gamma
        //     // m(1, 2) = y;// beta
        //     // m(2, 0) = 0;
        //     // m(2, 1) = 0;
        //     // m(2, 2) = 1;
        // }
    }

    void populatePositionMatrix() {
        int certainty, angle, distance;
        while (data >> certainty >> angle >> distance) {
            // m(lines, 0) = a;
            positionMatrix.conservativeResize(positionMatrix.rows() + 1, 3);
            // if (positionMatrix.rows() == 11) break;
            // m(m.rows() - 1, 0) = M_PI * angle / 180;
            // m(m.rows() - 1, 1) = distance;

            float angles_rad = fmod((-angle *(M_PI / 180)),2*M_PI);
            positionMatrix(positionMatrix.rows() - 1, 0) = distance * cos(angles_rad) / 10;
            positionMatrix(positionMatrix.rows() - 1, 1) = distance * sin(angles_rad) / 10;
            positionMatrix(positionMatrix.rows() - 1, 2) = 1;
            // std::cout << m(m.rows() - 1, 0) << " " << m(m.rows() - 1, 1) << std::endl;
            // std::cout << m << std::endl << std::endl;
        }
        
        // std::cout << positionMatrix << std::endl;
        display->addFigure(positionMatrix, ftxui::Color::Green);
        //screen(positionMatrix);
        // std::cout << "Matrix size " << m.rows() << " " << m.cols() << std::endl;
        // for (int c = 0; c < 5; c++) {
        //     for (int r = 0; r < 3; r++) {
        //         std::cout << positionMatrix(c,r) << " ";
        //     }
        //     std::cout << std::endl;
        // }
        // std::cout << "hellooo" << std::endl;
    }

    float calculate_median( std::vector<double> list){
        float median = 0.0;
        std::sort(list.begin(),list.end()); // sort the list with squared distances
        if (list.size() % 2 != 0) // if nr of elements in squared dists list is odd, choose the middle element as median
            median = list[ list.size()/ 2];
        else // else take averag of the two middle numbers in the list of squared distances
            median = list[(list.size() - 1) / 2] + list[list.size() / 2] / 2.0;
        return median;
    }

    void cox_linefit(/*std::vector<float> angle, std::vector<float> distance, std::vector<float> position*/) {
        //float ddx, ddy, dda = 0;
        int max_iterations = 10;

        Eigen::Matrix<double, 4, 2> unit_vectors;
        std::vector<double> line_distances;
        for(int i = 0; i < unit_vectors.rows(); i++) {
            // Eigen::Vector2d point1 = {line_model(i, 0), line_model(i, 1)};
            // Eigen::Vector2d point2 = {line_model(i, 2), line_model(i, 3)};
            Eigen::Vector2d point1;
            point1 << line_model(i, 0), line_model(i, 1);

            Eigen::Vector2d point2;
            point2 << line_model(i, 2), line_model(i, 3);

            Eigen::Vector2d Li = point2 - point1;
            Eigen::Vector2d moved_Li;
            moved_Li << -Li(1), Li(0);
            unit_vectors.row(i) = moved_Li / moved_Li.norm();
            //line_distances.push_back(point1.dot(unit_vectors.row(i)));
            line_distances.push_back(abs(unit_vectors.row(i).dot(point1)));
            // std::cout << line_distances[i] << std::endl;
        }
        // std::cout <<std::endl << unit_vectors << std::endl<<std::endl;
        
        Eigen::MatrixXd Xs(3, positionMatrix.rows());
        Eigen::MatrixXd Xwt(positionMatrix.rows(), 3);
        std::vector<double> assigned_line_index(positionMatrix.rows(),0); // store all indexes of the lines that corresponds to points in this list
        std::vector<double> squared_dists(positionMatrix.rows(),0);
        std::vector<double> all_yi(positionMatrix.rows(),0); // store all shortest distances to each point in this list
        Eigen::MatrixXd all_vi(positionMatrix.rows(), 2);
        for(int i = 0; i < max_iterations; i++) {
            Xs = RMatrix * positionMatrix.transpose();
            //std::cout << RMatrix << std::endl;
            // std::cout << positionMatrix << std::endl;
            Xwt = (CMatrix * Xs).transpose();
        
            all_vi = Xwt.block(0, 0, Xwt.rows(), 2);
            
    /*
            // connect all points to a line in these loops below
            for(int j = 0; j < all_vi.rows(); j++) //loop throguh all lidar points
            {
                float min_dist = 1000000000.0;
                float true_min_dist = min_dist;
                int closest_line_index = 0;


                for(int k = 0; k < line_model.rows(); k++) // loop through all lines
                {
                    float yi = line_distances[k] - unit_vectors.row(k).dot(all_vi.row(j));

                    if(abs(yi) < min_dist) // if we find a line that is closer than the previous closest line, update it
                    {
                        min_dist = abs(yi);
                        closest_line_index = k;
                        true_min_dist = yi;
                    }
                }

                assigned_line_index[j] = closest_line_index;
                squared_dists[j] = min_dist * min_dist;
                all_yi[j] = true_min_dist;
            }*/

            /*
            // update unitvectors list, so they follow the order of the assigned lines to each point
            Eigen::MatrixXd new_unitvectors(assigned_line_index.size(), 2);
            for (int j = 0; j < assigned_line_index.size(); j++)
            {
                new_unitvectors.row(j) = unit_vectors.row(assigned_line_index[j]);
            }

            // find median of squared distances:
            float median = calculate_median(squared_dists);

            Eigen::VectorXd all_yi_eigen(0);
            Eigen::MatrixXd all_vi_updated(0,2);
            Eigen::MatrixXd unit_vectors_updated(0,2);

            // remove all outliers from the lists. The outliers are all points which have a distance above the median to the closest line:
            for (int j = 0; j < squared_dists.size(); j++)
            {
                if (squared_dists[j] <= median) // if not an outlier, add them to new lists
                {
                    // update unitvectors
                    unit_vectors_updated.conservativeResize(unit_vectors_updated.rows() + 1, 2);
                    unit_vectors_updated(unit_vectors_updated.rows() - 1,0) = new_unitvectors(j,0);
                    unit_vectors_updated(unit_vectors_updated.rows() - 1,1) = new_unitvectors(j,1);
                    // update sensor coordinates (all_vi)
                    all_vi_updated.conservativeResize(all_vi_updated.rows() + 1, 2);
                    all_vi_updated(all_vi_updated.rows()-1,0) = all_vi(j,0);
                    all_vi_updated(all_vi_updated.rows()-1,1) = all_vi(j,1);
                    // update distances (yi)
                    all_yi_eigen.conservativeResize(all_yi_eigen.rows()+1);
                    all_yi_eigen(all_yi_eigen.rows()-1) = all_yi[j];
                }
            }
        
            Eigen::MatrixXd A(unit_vectors_updated.rows(),3);
        // A.resize(unit_vectors_updated.rows(), 3); // is this allocating memory? 
            //std::cout << "yeehaw" << std::endl;
            Eigen::Matrix2d rotation(2,2);
            rotation << 0, -1,
                        1, 0;
            
            // Vm = current robot coordinates: 
            Eigen::Vector2d Vm; 
            Vm << start_x,start_y;
        
            // split unit vectors in x-coordinate, y-coordinate and angle
            for (int j = 0; j < unit_vectors_updated.rows(); j++) 
            {
                A(j, 0) = unit_vectors_updated(j,0); // x coordinate
                A(j, 1) = unit_vectors_updated(j,1); // y coordinate
                float a = (unit_vectors_updated.row(j) * rotation).dot( all_vi_updated.row(j) - Vm.transpose());
                A(j, 2) = a; // angle
            }

            // create b matrix, which contains the correction of the position:
            Eigen::MatrixXd b;
            b = (A.transpose() * A).inverse() * A.transpose() * all_yi_eigen;
            //std::cout << b << std::endl;

            //update "overall congurnce" (how far it is from the original position of robot? :-) :-S):
            ddx = ddx + b(0);
            ddy = ddy + b(1);
            dda = dda + b(2);

            // update robot position:
            start_x = start_x + b(0);
            start_y = start_y + b(1);
            start_angle = fmod(start_angle + b(2), 2*M_PI); // need modulus 2*pi here maybe ? 

            // covariance matrix calculations (uncertainty): 
            int n = A.rows();
            float s2 = (all_yi_eigen-A*b).transpose().dot(all_yi_eigen -A*b) / (n-4);
            //std::cout << s2 << std::endl;
            Eigen::MatrixXd covariance_matrix; 
            covariance_matrix = s2 * (A.transpose() * A).inverse();
            //std::cout << covariance_matrix << std::endl;

            //check if the process has converged
            //std::cout << "convergeance number = " << sqrt(pow(b(0),2) + pow(b(1),2)) << std::endl;
            if (sqrt(pow(b(0),2) + pow(b(1),2)) < 0.3 ) // && abs(b(2)) < 0.1 * M_PI / 180
            {
                std::cout << "converged at iteration: " << i << std::endl;
                std::cout << "converged at robot position: Rx = " << start_x << ", Ry = " << start_y << ", Ra = " << start_angle << std::endl;
            // std::cout << "all_vi after converged: "<< all_vi_updated << std::endl;
                screen(positionMatrix,all_vi);   
                return;
            }
            */
        }

        
        // std::cout << "did not converge, iterated all loops of cox" << std::endl;
        display->addFigure(positionMatrix, ftxui::Color::Blue);
        // screen(positionMatrix,all_vi); 
        display->draw();
    }
};