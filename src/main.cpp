#include "../Eigen/Dense"

// #include <sys/socket.h>
// #include <netinet/in.h>
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>

#else
#include <unistd.h>
#endif
#include <iostream>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <mutex>
// #include <Eigen/Dense>

#define PORT 9888

// int originx = -1;
// int originy = -1;

// float y_start = 10.5;
// float x_start = 13.5;

// class Lidar {
//     Lidar() {

//     }
// }

// int main2() {
//     Eigen::MatrixXd m(3, 3);

//     ssize_t valread;
//     char buffer[1024] = {0};
//     int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
//     sockaddr_in serverAddress;
//     serverAddress.sin_family = AF_INET;
//     serverAddress.sin_port = htons(PORT);
//     serverAddress.sin_addr.s_addr = INADDR_ANY;
//     bind(serverSocket, (sockaddr *)&serverAddress, sizeof(serverAddress));
//     listen(serverSocket, 5);
//     int newSock = accept(serverSocket, NULL, NULL);

    
//     while (1) {
//         // Receive header
//         valread = read(newSock, buffer, 5);
//         if (valread != 5) break;

//         // Parse header
//         if (buffer[0] == (char)0xA5) {
//             unsigned int data_length = ((buffer[2] << 16) | (buffer[3] << 8) | buffer[4]) & 0xFFFFFF;

//             // Receive data
//             valread = read(newSock, buffer, data_length);
//             if (valread != data_length) break;

//             unsigned int angle = (((unsigned int)buffer[1] >> 1) + (((unsigned int)buffer[2]) << 7)) >> 6;
//             unsigned int distance = (((unsigned int)buffer[3]) + (((unsigned int)buffer[4]) << 8)) >> 2;
//             unsigned char quality = buffer[0] >> 2;

//             std::cout << static_cast<int>(quality) << " " << angle << " " << distance << std::endl;

//             int angles_rad = M_PI * angle / 180;

//             // float x = distance * cos(angles_rad);
//             // float y = distance * sin(angles_rad);
//             //std::cout << static_cast<int>(quality) << " " << angle << " " << distance << " " << x << " " << y << std::endl;
            
//             // m(0, 0) = cos();
//             // m(0, 1) = -sin();
//             // m(0, 2) = x;//S alpha
//             // m(1, 0) = sin();//_gamma
//             // m(1, 1) = cos();//_gamma
//             // m(1, 2) = y;// beta
//             // m(2, 0) = 0;
//             // m(2, 1) = 0;
//             // m(2, 2) = 1;
//         }
//     }

//     close(newSock);
//     close(serverSocket);

//     return 0;
// }

    // 
    // old_x = x
    // new_y = y
    // moved_x = old_x - x
    // moved_y = old_y - y





    // Eigen::Matrix<double, 4, 2> line_ref;
    // line_ref << 0,  0,
    //           0, 57,
    //           27,57,
    //           27,0;

    // Eigen::Vector2d point1 = line_ref.row(0);
    // Eigen::Vector2d point2 = line_ref.row(1);

    // Eigen::Vector2d Li = point2 - point1;
    // // std::cout << Li << " Li " << std::endl;

    // Eigen::Vector2d moved_Li;
    // moved_Li << Li(0), Li(1); //maybe add - to first element 
    // // std::cout << moved_Li << "moved_Li" << std::endl;
    
    // Eigen::Vector2d unit_vectors = moved_Li / moved_Li.norm();
   
    // std::cout << std::endl << unit_vectors << "unit vectors" << std::endl;

    // // Eigen::Vector2d random_point = { 17-x_start, 57-y_start };
    // // Eigen::Vector2d random_point = { 17-x_start, 57-y_start+10 };
    // Eigen::Vector2d random_point = { 30, 0 };
    // std::cout << std::endl << random_point << "random_point" << std::endl;

    // Eigen::Vector2d origin_v = { x_start, y_start };
    // std::cout << std::endl << origin_v << "origin_v" << std::endl;

    // Eigen::Vector2d random_point_v = random_point + origin_v;

    // std::cout << std::endl << random_point_v << "random_point_v" << std::endl;

    // Eigen::Vector2d dist1 = (random_point_v . dot(unit_vectors) / unit_vectors.norm() ) * unit_vectors;
    // unit_vectors = (Eigen::Vector2d){Li(1), Li(0)} / (Eigen::Vector2d){Li(1), Li(0)}.norm();
    // Eigen::Vector2d dist2 = (random_point_v . dot(unit_vectors) / unit_vectors.norm() ) * unit_vectors;
    // std::cout << std::endl << std::endl << dist1 << "dist1" << std::endl;
    // std::cout << std::endl << std::endl << dist2 << "dist2" << std::endl;
    // // Eigen::Vector2d dist = (dist1. < dist2) ? dist1 : dist2;

    // Eigen::Vector2d error_pt1 = dist1 - Li;
    // Eigen::Vector2d error_pt2 = dist2 - (Eigen::Vector2d){27, 0};
    // std::cout << std::endl << std::endl << error_pt1 << "error_pt1" << std::endl;
    // std::cout << std::endl << std::endl << error_pt2 << "error_pt2" << std::endl;
    // Eigen::Vector2d error_pt = (error_pt1[0] < error_pt2[1]) ? error_pt1 : error_pt2;
    // error_pt = error_pt.cwiseAbs();

    // std::cout << std::endl << error_pt << "error point" << std::endl;






    //return 0;
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "./include.hpp"

int alpha, beta = 0;
int _gamma = 0;//(-180 * M_PI / 180);
double start_x = 149; // to be changed when start
double start_y = 87.5; // to be changed when start
double start_angle = 0;//90*M_PI / 180;
double ddx = 0;
double ddy = 0;
double dda = 0;
Eigen::Matrix<double, 4, 4> line_model;
Eigen::MatrixXd RMatrix(3, 3);
Eigen::MatrixXd CMatrix(3, 3);
Eigen::MatrixXd positionMatrixPool(360, 3);
Eigen::MatrixXd positionMatrix(360, 3);

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
    ddx, ddy, dda = 0;
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
    //std::cout << "unit vectors:" << unit_vectors << std::endl;
    Eigen::MatrixXd temp(positionMatrix.rows(),3);
    Eigen::MatrixXd Xs(3, positionMatrix.rows());
    
    Eigen::MatrixXd Xw(3,positionMatrix.rows());
    Eigen::MatrixXd Xwt(positionMatrix.rows(), 3);
    std::vector<double> assigned_line_index(positionMatrix.rows(),0); // store all indexes of the lines that corresponds to points in this list
    std::vector<double> squared_dists(positionMatrix.rows(),0);
    std::vector<double> all_yi(positionMatrix.rows(),0); // store all shortest distances to each point in this list
    Eigen::MatrixXd all_vi(positionMatrix.rows(), 2);
    
    for(int i = 0; i < max_iterations; i++) {
        RMatrix <<  cos(_gamma), -sin(_gamma), alpha,
                sin(_gamma), cos(_gamma), beta,
                0, 0, 1;

        CMatrix <<  cos(start_angle), -sin(start_angle), start_x,
                sin(start_angle), cos(start_angle), start_y,
                0, 0, 1;

        Xs = RMatrix * positionMatrix.transpose();
        //std::cout << RMatrix << std::endl;
      
        Xw = (CMatrix * Xs);
        Xwt =Xw.transpose();
        //std::cout << Xwt << std::endl;
        if(i == 0)
            temp = Xwt;

        all_vi = Xwt.block(0, 0, Xwt.rows(), 2);
        

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
        }

      
        // update unitvectors list, so they follow the order of the assigned lines to each point
        Eigen::MatrixXd new_unitvectors(assigned_line_index.size(), 2);
        for (int j = 0; j < assigned_line_index.size(); j++)
        {
            new_unitvectors.row(j) = unit_vectors.row(assigned_line_index[j]);
        }

        //std::cout << "unit vectors correspoind to dots: " << new_unitvectors << std::endl;

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
            std::cout << "ddx = " << ddx << ", ddy = " << ddy << ", dda = " << dda << std::endl;

        // update robot position:
        start_x = start_x + b(0);
        start_y = start_y + b(1);
        //start_angle = fmod(start_angle + b(2), 2*M_PI); // need modulus 2*pi here maybe ? 
        start_angle = start_angle + b(2);
        // covariance matrix calculations (uncertainty): 
        int n = A.rows();
        float s2 = (all_yi_eigen-A*b).transpose().dot(all_yi_eigen -A*b) / (n-4);
        //std::cout << s2 << std::endl;
        Eigen::MatrixXd covariance_matrix; 
        covariance_matrix = s2 * (A.transpose() * A).inverse();
        //std::cout << covariance_matrix << std::endl;

        //check if the process has converged

 /**/
        if (sqrt(pow(b(0),2) + pow(b(1),2)) < 0.001 ) // && abs(b(2)) < 0.1 * M_PI / 180
        {
        std::cout << "convergeance number = " << sqrt(pow(b(0),2) + pow(b(1),2)) << std::endl;
            std::cout << "converged at iteration: " << i << std::endl;
            std::cout << "robot position: Rx = " << start_x << ", Ry = " << start_y << ", Ra = " << start_angle << std::endl;
            std::cout << "ddx = " << ddx << ", ddy = " << ddy << ", dda = " << dda << std::endl;
            std::cout << "covariance matrix: " << covariance_matrix << std::endl;
           // std::cout << "all_vi after converged: "<< all_vi_updated << std::endl;
            screen(temp,all_vi_updated);   
            return;
        }
   
    }
   
    std::cout << "did not converge, iterated all loops of cox" << std::endl;
    screen(temp,all_vi); 
    
}

int lines = 0;
std::mutex mtx;

void data_thread() {
    ssize_t valread;
    char buffer2[1024] = {0};
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(PORT);
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    bind(serverSocket, (sockaddr *)&serverAddress, sizeof(serverAddress));
    listen(serverSocket, 5);
    int newSock = accept(serverSocket, NULL, NULL);
    
    while(true) {
        // for(int count = 0; count < 360; count++) {
            // Receive header
            valread = read(newSock, buffer2, 5);
            if (valread != 5) continue;//throw std::runtime_error("Could not read header");

            // Parse header
            if (buffer2[0] == (char)0xA5) {
                unsigned int data_length = ((buffer2[2] << 16) | (buffer2[3] << 8) | buffer2[4]) & 0xFFFFFF;

                // Receive data
                valread = read(newSock, buffer2, data_length);
                if (valread != data_length) break;

                unsigned int angle = (((unsigned int)buffer2[1] >> 1) + (((unsigned int)buffer2[2]) << 7)) >> 6;
                unsigned int distance = (((unsigned int)buffer2[3]) + (((unsigned int)buffer2[4]) << 8)) >> 2;
                unsigned char quality = buffer2[0] >> 2;
            float angles_rad = fmod((angle *(M_PI / 180)),2*M_PI);
            // positionMatrix(positionMatrix.rows() - 1, 0) = distance * cos(angles_rad) / 10;
            // positionMatrix(positionMatrix.rows() - 1, 1) = distance * sin(angles_rad) / 10;
            // positionMatrix(positionMatrix.rows() - 1, 2) = 1;
            if (lines == 359) {
                positionMatrixPool = positionMatrixPool.block(1, 0, positionMatrixPool.rows() - 1, 3);
                positionMatrixPool.conservativeResize(positionMatrixPool.rows() + 1, 3); // to be changed to fixed size so we don't have to resize
                // std::cout << "matrix size " << positionMatrixPool.rows() << std::endl;
            }
            mtx.lock();
            positionMatrixPool(lines, 0) = distance * cos(-angles_rad) / 10;
            positionMatrixPool(lines, 1) = distance * sin(-angles_rad) / 10;
            positionMatrixPool(lines, 2) = 1;
            if (lines < 359) lines++;
            mtx.unlock();
        }
    }
}

#include <thread>

int main(void) {
    // std::ifstream data ("./newdata4.txt");
    std::vector<int> buffer;
    // Eigen::matrix<double, 
    // sensor coordinates to robot coordinates
    RMatrix <<  cos(_gamma), -sin(_gamma), alpha,
                sin(_gamma), cos(_gamma), beta,
                0, 0, 1;
    // robot to world coordinates
    CMatrix <<  cos(start_angle), -sin(start_angle), start_x,
                sin(start_angle), cos(start_angle), start_y,
                0, 0, 1;

    // line model to change if box changes
    line_model <<  0, 0, 0, 365,
                0, 365, 244, 365,
                244, 365, 244, 0,
                244, 0, 0, 0;


    // if (!data.is_open()) throw std::runtime_error("Could not open file");
    std::thread data_th(data_thread);
    data_th.detach();
    // data_th.join();
    // return 0;

    while (lines < 359); // wait for first batch of data


    while (true) {
        mtx.lock();
        positionMatrix = positionMatrixPool; // copy the data from the pool to the matrix
        //print last 5 lines of matrix
        // std::cout << positionMatrixPool.block(positionMatrixPool.rows() - 5, 0, 5, 3) << std::endl;
        mtx.unlock();
        cox_linefit(); // run the cox linefit algorithm on the dynamic data
        // for (int i = 0; i < 50000000; i++);
    }

}
