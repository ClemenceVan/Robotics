#pragma once
#include "../pch.h"

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
        Arena(std::pair<int, int> size, std::pair<double, double> origin);

        Eigen::MatrixXd getLineModel();

        std::pair<int, int> getSize();

        std::pair<double, double> getOrigin();
};