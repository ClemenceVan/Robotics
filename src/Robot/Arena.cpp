#include "Arena.hpp"

Arena::Arena(std::pair<int, int> size, std::pair<double, double> origin) {
    this->width = std::get<0>(size);
    this->height = std::get<1>(size);
    this->originX = std::get<0>(origin);
    this->originY = std::get<1>(origin);
    this->line_model <<
        0, 0, 0, height,
        0, height, width, height,
        width, height, width, 0,
        width, 0, 0, 0;
}

Eigen::MatrixXd Arena::getLineModel() {
    return this->line_model;
}

std::pair<int, int> Arena::getSize() {
    return std::make_pair(this->width, this->height);
}

std::pair<double, double> Arena::getOrigin() {
    return std::make_pair(this->originX, this->originY);
}