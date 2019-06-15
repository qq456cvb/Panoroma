//
//  RANSAC.cpp
//  Panoroma
//
//  Created by Neil on 2019/6/15.
//  Copyright © 2019 Neil. All rights reserved.
//

#include <Eigen/SVD>
#include "RANSAC.hpp"
Eigen::Matrix3f RANSACHomoSolver::solve(const vector<Point> &pts1, const vector<Point> &pts2) {
    srand(time(nullptr));
    Eigen::Matrix3f best_homo;
    Eigen::MatrixXf A(2 * min_corres, 9);
    int best_match = std::numeric_limits<int>::min();
    for (size_t i = 0; i < n_trial; ++i) {
        int inlier_cnt = 0;
        for (size_t j = 0; j < min_corres; ++j) {
            int rnd_idx = rand() % pts1.size();
            
            A(2 * j, 0) = 0;
            A(2 * j, 1) = 0;
            A(2 * j, 2) = 0;
            A(2 * j, 3) = -pts2[rnd_idx].x;
            A(2 * j, 4) = -pts2[rnd_idx].y;
            A(2 * j, 5) = -1;
            A(2 * j, 6) = pts1[rnd_idx].y * pts2[rnd_idx].x;
            A(2 * j, 7) = pts1[rnd_idx].y * pts2[rnd_idx].y;
            A(2 * j, 8) = pts1[rnd_idx].y;
            
            A(2 * j + 1, 0) = pts2[rnd_idx].x;
            A(2 * j + 1, 1) = pts2[rnd_idx].y;
            A(2 * j + 1, 2) = 1;
            A(2 * j + 1, 3) = 0;
            A(2 * j + 1, 4) = 0;
            A(2 * j + 1, 5) = 0;
            A(2 * j + 1, 6) = -pts1[rnd_idx].x * pts2[rnd_idx].x;
            A(2 * j + 1, 7) = -pts1[rnd_idx].x * pts2[rnd_idx].y;
            A(2 * j + 1, 8) = -pts1[rnd_idx].x;
        }
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV);
        Eigen::VectorXf tmp = svd.matrixV().rightCols(1);
        Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> h(tmp.data());
        for (size_t k = 0; k < pts2.size(); ++k) {
            Eigen::Vector3f p2;
            p2 << pts2[k].x, pts2[k].y, 1;
            Eigen::Vector3f p1_est = h * p2;
            float x = p1_est.x() / p1_est.z(), y = p1_est.y() / p1_est.z();
            float err = sqrt((x - pts1[k].x) * (x - pts1[k].x) + (y - pts1[k].y) * (y - pts1[k].y));
            if (err < err_thresh) {
                inlier_cnt++;
            }
        }
        if (inlier_cnt > best_match) {
            std::cout << "found best with inlier cnt: " << inlier_cnt << std::endl;
            best_homo = h;
        }
    }
    return best_homo;
}
