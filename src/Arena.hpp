#pragma once
#include "./include.hpp"

class Arena {
    private:
    int width;
    int height;

    double originX;
    double originY;

    Eigen::MatrixXd line_model = Eigen::MatrixXd(4, 4);
    public:
    /**
     * @brief Construct a new Arena object
     * 
     * @param size { width, height }
     * @param origin { x, y }
     */
    Arena(std::pair<int, int> size, std::pair<double, double> origin) {
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

    Eigen::MatrixXd getLineModel() {
        return this->line_model;
    }

    std::pair<int, int> getSize() {
        return std::make_pair(this->width, this->height);
    }

    std::pair<double, double> getOrigin() {
        return std::make_pair(this->originX, this->originY);
    }
};