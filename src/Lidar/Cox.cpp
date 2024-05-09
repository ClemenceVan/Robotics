#include "Lidar.hpp"


float Lidar::calculate_median(std::vector<double> list) {
    float median = 0.0;
    // sort the list with squared distances
    std::sort(list.begin(),list.end());
    // if nr of elements in squared dists list is odd, choose the middle element as median
    if (list.size() % 2 != 0)
        median = list[ list.size()/ 2];
    else // else take averag of the two middle numbers in the list of squared distances
        median = list[(list.size() - 1) / 2] + list[list.size() / 2] / 2.0;
    return median;
}

void Lidar::cox_linefit() {
    this->ddx, this->ddy, this->dda = 0;
    int max_iterations = 10;

    this->positionMatrix = this->getData();

    Eigen::Matrix<double, 4, 2> unit_vectors;
    std::vector<double> line_distances;
    for(int i = 0; i < unit_vectors.rows(); i++) {
        Eigen::Vector2d point1;
        point1 << this->line_model(i, 0), this->line_model(i, 1);

        Eigen::Vector2d point2;
        point2 << this->line_model(i, 2), this->line_model(i, 3);

        Eigen::Vector2d Li = point2 - point1;
        Eigen::Vector2d moved_Li;
        moved_Li << -Li(1), Li(0);
        unit_vectors.row(i) = moved_Li / moved_Li.norm();
        line_distances.push_back(abs(unit_vectors.row(i).dot(point1)));
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
    
        Xw = (this->CMatrix * Xs);
        Xwt = Xw.transpose();
        if(i == 0)
            temp = Xwt;
        all_vi = Xwt.block(0, 0, Xwt.rows(), 2);
        

        // connect all points to a line in these loops below
        for(int j = 0; j < all_vi.rows(); j++) { //loop throguh all lidar points
            float min_dist = 1000000000.0;
            float true_min_dist = min_dist;
            int closest_line_index = 0;

            for(int k = 0; k < this->line_model.rows(); k++) { // loop through all this->lines
                float yi = line_distances[k] - unit_vectors.row(k).dot(all_vi.row(j));
                if(abs(yi) < min_dist) { // if we find a line that is closer than the previous closest line, update it
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
            new_unitvectors.row(j) = unit_vectors.row(assigned_line_index[j]);

        // find median of squared distances:
        float median = calculate_median(squared_dists);

        Eigen::VectorXd all_yi_eigen(0);
        Eigen::MatrixXd all_vi_updated(0,2);
        Eigen::MatrixXd unit_vectors_updated(0,2);
    
        // remove all outliers from the lists. The outliers are all points which have a distance above the median to the closest line:
        for (int j = 0; j < squared_dists.size(); j++) {
            if (squared_dists[j] <= median) { // if not an outlier, add them to new lists
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
            A(j, 2) = a; // angle
        }

        // create b matrix, which contains the correction of the position:
        Eigen::MatrixXd b;
        b = (A.transpose() * A).inverse() * A.transpose() * all_yi_eigen;

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
        Eigen::MatrixXd covariance_matrix;  // to be moved for use with kalman
        covariance_matrix = s2 * (A.transpose() * A).inverse();
        //std::cout << covariance_matrix << std::endl;

        // check if the process has converged
        if (sqrt(pow(b(0),2) + pow(b(1),2)) < 0.001 ) // && abs(b(2)) < 0.1 * M_PI / 180
        {
            //std::cout << "convergeance number = " << sqrt(pow(b(0),2) + pow(b(1),2)) << std::endl;
            //std::cout << "converged at iteration: " << i << std::endl;
            //std::cout << "this->ddx = " << this->ddx << ", this->ddy = " << this->ddy << ", this->dda = " << this->dda << std::endl;
            std::cout << std::endl << "robot position: Rx = " << this->posX << ", Ry = " << this->posY << ", Ra = " << this->start_angle * 180 / M_PI << std::endl;
            //std::cout << "covariance matrix: " << covariance_matrix << std::endl;
            //std::cout << "all_vi after converged: "<< all_vi_updated << std::endl;
            screen(temp,all_vi_updated);  
            // display->updatePositions(temp, all_vi_updated);
            return;
        }

    }
    
    //std::cout << "did not converge, iterated all loops of cox" << std::endl;
            std::cout << std::endl << "robot position: Rx = " << this->posX << ", Ry = " << this->posY << ", Ra = " << this->start_angle * 180 / M_PI << std::endl;
    screen(temp,all_vi); 
    // display->updatePositions(temp, all_vi);
    
}