#include "./include.hpp"
#include "./Display.hpp"
#include "./Arena.hpp"
// #include "asio/include/asio.hpp"
// #include <asio.hpp>



class Lidar {
    private:
    Arena *arena;
    int alpha = 0;
    int beta = 0;
    int _gamma = (-90 * (int)M_PI / 180) / 4;
    double posX = 13.5; // to be changed when start
    double posY = 10.5; // to be changed when start
    int start_angle = 0;
    double ddx = 0;
    double ddy = 0;
    double dda = 0;
    Eigen::MatrixXd RMatrix = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd CMatrix = Eigen::MatrixXd::Zero(3, 3);
    // Eigen::MatrixXd positionMatrix = Eigen::MatrixXd::Zero(3, 1);
    Eigen::MatrixXd positionMatrixPoll = Eigen::MatrixXd::Zero(360, 3);
    Eigen::MatrixXd positionMatrix = Eigen::MatrixXd::Zero(1, 3);
    Eigen::Matrix<double, 4, 4> line_model;
    int lines = 0;

    std::mutex mtx;

    Display *display;

    bool _isPolling = false;
    ssize_t valread;
    char buffer2[1024] = {0};
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in serverAddress;
    int newSock;

    std::thread pollingThread;

    public:
    Lidar(Arena arena, bool display = true) {
        
        this->arena = &arena;

        this->line_model = arena.getLineModel();
        this->posX = std::get<0>(arena.getOrigin());
        this->posY = std::get<1>(arena.getOrigin());
        this->RMatrix <<  cos(this->_gamma), -sin(this->_gamma), this->alpha,
                        sin(this->_gamma), cos(this->_gamma), this->beta,
                        0, 0, 1;
        // robot to world coordinates
        this->CMatrix <<  cos(this->start_angle), -sin(this->start_angle), this->posX,
                    sin(this->start_angle), cos(this->start_angle), this->posY,
                    0, 0, 1;

        this->serverAddress.sin_family = AF_INET;
        this->serverAddress.sin_port = htons(PORT);
        this->serverAddress.sin_addr.s_addr = INADDR_ANY;
        bind(this->serverSocket, (struct sockaddr *)&this->serverAddress, sizeof(this->serverAddress));
        listen(this->serverSocket, 5);
        newSock = accept(this->serverSocket, NULL, NULL);


        std::cout << "New connection from: " << newSock << std::endl;
        // this->pollLidarData();
        pollingThread = std::thread(&Lidar::pollLidarData, this);
        pollingThread.detach();
        
            // io_context.run();
        if (display) {
            this->display = new Display({100, 100}, arena);
            this->display->draw();
        }
    }

    ~Lidar() {
        if (_isPolling) {
            close(this->newSock);
            close(this->serverSocket);
            // this->socket.close();
        }
    }

    int isDataReady() {
        mtx.lock();
        int l = this->lines;
        mtx.unlock();
        if (l >= 359) return 1;
        return 0;
    }

    Eigen::MatrixXd getData() {
        mtx.lock();
        Eigen::MatrixXd m = this->positionMatrixPoll;
        mtx.unlock();
        return m;
    }

    void pollLidarData() {
        // serverAddress.sin_family = AF_INET;
        // serverAddress.sin_port = htons(PORT);
        // serverAddress.sin_addr.s_addr = INADDR_ANY;
        // bind(serverSocket, (sockaddr *)&serverAddress, sizeof(serverAddress));
        // listen(serverSocket, 5);
        // int newSock = accept(serverSocket, NULL, NULL);
        
        while(true) {
            // for(int count = 0; count < 360; count++) {
                // Receive header
                valread = read(newSock, buffer2, 5);
                if (valread != 5) throw std::runtime_error("Could not read header");

                // Parse header
                if (buffer2[0] == (char)0xA5) {
                    unsigned int data_length = ((buffer2[2] << 16) | (buffer2[3] << 8) | buffer2[4]) & 0xFFFFFF;

                    // Receive data
                    valread = read(newSock, buffer2, data_length);
                    if (valread != data_length) break;

                    unsigned int angle = (((unsigned int)buffer2[1] >> 1) + (((unsigned int)buffer2[2]) << 7)) >> 6;
                    unsigned int distance = (((unsigned int)buffer2[3]) + (((unsigned int)buffer2[4]) << 8)) >> 2;
                    unsigned char quality = buffer2[0] >> 2;
                    if (quality < 10 || distance == 0) continue;
                    // std::cout << static_cast<int>(quality) << " " << angle << " " << distance << std::endl;
                float angles_rad = fmod((angle *(M_PI / 180)),2*M_PI);
                // positionMatrix(positionMatrix.rows() - 1, 0) = distance * cos(angles_rad) / 10;
                // positionMatrix(positionMatrix.rows() - 1, 1) = distance * sin(angles_rad) / 10;
                // positionMatrix(positionMatrix.rows() - 1, 2) = 1;
                mtx.lock();
                if (lines == 359) {
                    // std::cout << "matrix size before " << positionMatrixPoll.rows() << std::endl;
                    // positionMatrixPoll = positionMatrixPoll.block(1, 0, positionMatrixPoll.rows() - 1, 3);
                    positionMatrixPoll.block(0, 0, positionMatrixPoll.rows() - 1, 3) = positionMatrixPoll.block(1, 0, positionMatrixPoll.rows() - 1, 3);
                    // positionMatrixPoll.conservativeResize(positionMatrixPoll.rows() + 1, 3); // to be changed to fixed size so we don't have to resize
                    // std::cout << "matrix size " << positionMatrixPoll.rows() << std::endl;
                }
                positionMatrixPoll(lines, 0) = distance * cos(-angles_rad) / 10;
                positionMatrixPoll(lines, 1) = distance * sin(-angles_rad) / 10;
                positionMatrixPoll(lines, 2) = 1;
                if (lines < 359) lines++;
                mtx.unlock();
            } else {
                // std::cout << "Invalid header" << std::endl;
                // throw std::runtime_error("Invalid header");
            }
        }
    }

    float calculate_median(std::vector<double> list){
        float median = 0.0;
        std::sort(list.begin(),list.end()); // sort the list with squared distances
        if (list.size() % 2 != 0) // if nr of elements in squared dists list is odd, choose the middle element as median
            median = list[ list.size()/ 2];
        else // else take averag of the two middle numbers in the list of squared distances
            median = list[(list.size() - 1) / 2] + list[list.size() / 2] / 2.0;
        return median;
    }

    void cox_linefit() {
        this->ddx, this->ddy, this->dda = 0;
        int max_iterations = 1; // 10

        this->positionMatrix = this->getData();
        // this->display->addFigure(this->positionMatrix, ftxui::Color::Red);
        // std::cout <<"this->line_model: " << this->line_model << std::endl;

        Eigen::Matrix<double, 4, 2> unit_vectors;
        std::vector<double> line_distances;
        for(int i = 0; i < unit_vectors.rows(); i++) {
            // Eigen::Vector2d point1 = {this->line_model(i, 0), this->line_model(i, 1)};
            // Eigen::Vector2d point2 = {this->line_model(i, 2), this->line_model(i, 3)};
            Eigen::Vector2d point1;
            point1 << this->line_model(i, 0), this->line_model(i, 1);

            Eigen::Vector2d point2;
            point2 << this->line_model(i, 2), this->line_model(i, 3);

            Eigen::Vector2d Li = point2 - point1;
            Eigen::Vector2d moved_Li;
            moved_Li << -Li(1), Li(0);
            unit_vectors.row(i) = moved_Li / moved_Li.norm();
            //line_distances.push_back(point1.dot(unit_vectors.row(i)));
            line_distances.push_back(abs(unit_vectors.row(i).dot(point1)));
            // std::cout << line_distances[i] << std::endl;
        }
        Eigen::MatrixXd temp(this->positionMatrix.rows(),3);
        Eigen::MatrixXd Xs(3, this->positionMatrix.rows());
        
        Eigen::MatrixXd Xw(3,this->positionMatrix.rows());
        Eigen::MatrixXd Xwt(this->positionMatrix.rows(), 3);
        std::vector<double> assigned_line_index(this->positionMatrix.rows(),0); // store all indexes of the this->lines that corresponds to points in this list
        std::vector<double> squared_dists(this->positionMatrix.rows(),0);
        std::vector<double> all_yi(this->positionMatrix.rows(),0); // store all shortest distances to each point in this list
        Eigen::MatrixXd all_vi(this->positionMatrix.rows(), 2);
        
        for(int i = 0; i < max_iterations; i++) {
            this->RMatrix <<  cos(this->_gamma), -sin(this->_gamma), this->alpha,
                    sin(this->_gamma), cos(this->_gamma), this->beta,
                    0, 0, 1;

            this->CMatrix <<  cos(this->start_angle), -sin(this->start_angle), this->posX,
                    sin(this->start_angle), cos(this->start_angle), this->posY,
                    0, 0, 1;

            Xs = this->RMatrix * this->positionMatrix.transpose();
            // std::cout << this->positionMatrix << std::endl;
        
            Xw = (this->CMatrix * Xs);
            Xwt = Xw.transpose();
            //std::cout << Xwt << std::endl;
            if(i == 0)
                temp = Xwt;

            all_vi = Xwt.block(0, 0, Xwt.rows(), 2);
            std::cout << "positionMatrix: "<< this->RMatrix << std::endl;
            // std::cout << "x : " << this->posX << ", y : " << this->posY << ", angle : " << this->start_angle << std::endl;
            

            // connect all points to a line in these loops below
            for(int j = 0; j < all_vi.rows(); j++) //loop throguh all lidar points
            {
                float min_dist = 1000000000.0;
                float true_min_dist = min_dist;
                int closest_line_index = 0;


                for(int k = 0; k < this->line_model.rows(); k++) // loop through all this->lines
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

        
            // update unitvectors list, so they follow the order of the assigned this->lines to each point
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
            // std::cout << "all_vi: "<< all_vi << std::endl;

        
            Eigen::MatrixXd A(unit_vectors_updated.rows(),3);
        // A.resize(unit_vectors_updated.rows(), 3); // is this allocating memory? 
            //std::cout << "yeehaw" << std::endl;
            Eigen::Matrix2d rotation(2,2);
            rotation << 0, -1,
                        1, 0;
            
            // Vm = current robot coordinates: 
            Eigen::Vector2d Vm; 
            Vm << this->posX,this->posY;
        
            // split unit vectors in x-coordinate, y-coordinate and angle
            for (int j = 0; j < unit_vectors_updated.rows(); j++) 
            {
                A(j, 0) = unit_vectors_updated(j,0); // x coordinate
                A(j, 1) = unit_vectors_updated(j,1); // y coordinate
                float a = (unit_vectors_updated.row(j) * rotation).dot( all_vi_updated.row(j) - Vm.transpose());
                A(j, 2) = a; // angle // rework: issue there -> all_vi_updated 0 = overflow
            }
            // std::cout << "all_vi_updated: " << all_vi_updated << std::endl;

            // create b matrix, which contains the correction of the position:
            Eigen::MatrixXd b;
            b = (A.transpose() * A).inverse() * A.transpose() * all_yi_eigen;
            /*b:
                -4.50903e+08
                -3.27255
                0
            */

            //update "overall congurnce" (how far it is from the original position of robot? :-) :-S):
            this->ddx = this->ddx + b(0);
            this->ddy = this->ddy + b(1);
            this->dda = this->dda + b(2);
                // std::cout << "this->ddx = " << this->ddx << ", this->ddy = " << this->ddy << ", this->dda = " << this->dda << std::endl;

            // update robot position:
            this->posX = this->posX + b(0);
            this->posY = this->posY + b(1);
            //this->start_angle = fmod(this->start_angle + b(2), 2*M_PI); // need modulus 2*pi here maybe ? 
            this->start_angle = this->start_angle + b(2);
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
                std::cout << "this->ddx = " << this->ddx << ", this->ddy = " << this->ddy << ", this->dda = " << this->dda << std::endl;
                std::cout << "robot position: Rx = " << this->posX - 14 << ", Ry = " << this->posY - 29 << ", Ra = " << this->start_angle * 180 / M_PI << std::endl;
                std::cout << "covariance matrix: " << covariance_matrix << std::endl;
            std::cout << "all_vi after converged: "<< all_vi_updated << std::endl;
                // screen(temp,all_vi_updated);   
                return;
            }
    
        }
    
        std::cout << "did not converge, iterated all loops of cox" << std::endl;
                std::cout << "robot position: Rx = " << this->posX - 14 << ", Ry = " << this->posY - 29 << ", Ra = " << this->start_angle * 180 / M_PI << std::endl;
        // screen(temp,all_vi); 
        
    }
};