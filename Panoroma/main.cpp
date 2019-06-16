//
//  main.cpp
//  Panoroma
//
//  Created by Neil on 07/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#include <iostream>
#include <algorithm>
#include "Mat2d.hpp"
#include "MatXd.hpp"
#include "imgio.hpp"
#include "imgproc.hpp"
#include "SIFT.hpp"
#include "RANSAC.hpp"
using namespace std;

int main(int argc, const char * argv[]) {
//    Mat3d<unsigned char> img(300, 200, 3);
    
    auto img1 = readImage("test1.jpeg");
    auto img2 = readImage("test2.jpeg");
//    Mat3d<unsigned char> lena(300, 200, 3);
//    lena.set_all(128);
//    Image draw = lena.clone();
//    drawCircle(draw, Point(20, 20), BLUE, 10);
//    imageShow(draw);
    img1 = rgb2gray(img1);
    img2 = rgb2gray(img2);
//    imageShow(lena);
//    auto blurred = gaussianBlur(lena, 1.3);
//    lena = downSample(lena, 1.5);
//    imageShow(lena);
//    lena = upSample(lena, 2);
//    imageShow(lena);
//    GaussianPyramid gp;
//    gp.build(lena);
//    imageShow(lena);
    Mat2d<unsigned char> descriptors1, descriptors2;
    SIFT sift1, sift2;
    sift1.extract(img1, descriptors1);
    sift2.extract(img2, descriptors2);
    vector<int> matches;
    // brute-force match
    Mat2d<float> dists(descriptors1.n_rows(), descriptors2.n_rows());
    for (int i = 0; i < descriptors1.n_rows(); i++) {
        for (int j = 0; j < descriptors2.n_rows(); j++) {
            float dist = 0.f;
            for (int k = 0; k < descriptors1.n_cols(); k++) {
                dist += (descriptors1[i][k] - descriptors2[j][k]) * (descriptors1[i][k] - descriptors2[j][k]);
            }
            dists[i][j] = std::sqrtf(dist);
        }
        float smallest = dists[i][0], second_smallest = dists[i][1];
        int smallest_idx = 0;
        if (second_smallest < smallest) {
            std::swap(smallest, second_smallest);
            smallest_idx = 1;
        }
        for (int k = 0; k < dists.n_cols(); k++) {
            if (dists[i][k] < smallest) {
                second_smallest = smallest;
                smallest = dists[i][k];
                smallest_idx = k;
            } else if (dists[i][k] < second_smallest) {
                second_smallest = dists[i][k];
            }
        }
        if (smallest < 0.75 * second_smallest) {
            matches.push_back(smallest_idx);
        } else {
            matches.push_back(-1);
        }
    }
    imageShow(drawMatches(img1, img2, sift1, sift2, matches));
    vector<Point> pts1, pts2;
//    pts1.emplace_back(0.f, 0.f);
//    pts1.emplace_back(1.f, 0.f);
//    pts1.emplace_back(1.f, 1.f);
//    pts1.emplace_back(0.f, 1.f);
//
//    pts2.emplace_back(0.f, 0.f);
//    pts2.emplace_back(1.f, 0.5f);
//    pts2.emplace_back(1.f, 1.f);
//    pts2.emplace_back(0.f, 0.5f);
    for (size_t i = 0; i < matches.size(); ++i) {
        if (matches[i] >= 0) {
            pts1.emplace_back(sift1.key_points_[i].p.x * powf(SCALE_INTERVAL, sift1.key_points_[i].octave) / img2.n_cols(), sift1.key_points_[i].p.y * powf(SCALE_INTERVAL, sift1.key_points_[i].octave) / img2.n_rows());
            pts2.emplace_back(sift2.key_points_[matches[i]].p.x * powf(SCALE_INTERVAL, sift2.key_points_[matches[i]].octave) / img2.n_cols(), sift2.key_points_[matches[i]].p.y * powf(SCALE_INTERVAL, sift2.key_points_[matches[i]].octave) / img2.n_rows());
        }
    }
    auto ransac = RANSACHomoSolver();
    auto homo = ransac.solve(pts1, pts2);
    std::cout << homo << std::endl;
    Mat2d<float> corners[4] = {Mat2d<float>(3 ,1), Mat2d<float>(3 ,1), Mat2d<float>(3 ,1), Mat2d<float>(3 ,1)};
    corners[0][0][0] = 0.f;
    corners[0][1][0] = 0.f;
    corners[0][2][0] = 1.f;
    
    corners[1][0][0] = 0.f;
    corners[1][1][0] = 1.f;
    corners[1][2][0] = 1.f;
    
    corners[2][0][0] = 1.f;
    corners[2][1][0] = 1.f;
    corners[2][2][0] = 1.f;
    
    corners[3][0][0] = 1.f;
    corners[3][1][0] = 0;
    corners[3][2][0] = 1.f;
    int min_x = std::numeric_limits<int>::max();
    int min_y = std::numeric_limits<int>::max();
    int max_x = std::numeric_limits<int>::min();
    int max_y = std::numeric_limits<int>::min();
    for (size_t i = 0; i < 4; ++i) {
        corners[i] = homo * corners[i];
        corners[i][0][0] /= corners[i][2][0];
        corners[i][1][0] /= corners[i][2][0];
        min_x = min(min_x, int(corners[i][0][0] * img2.n_cols() - 1));
        min_y = min(min_y, int(corners[i][1][0] * img2.n_rows() - 1));
        max_x = max(max_x, int(corners[i][0][0] * img2.n_cols() + 1));
        max_y = max(max_y, int(corners[i][1][0] * img2.n_rows() + 1));
        std::cout << corners[i] << std::endl;
    }
    Image Hx(max(max_y, img1.n_rows()) - min(min_y, 0), max(max_x, img1.n_cols()) - min(min_x, 0), 1);
    for (size_t i = 0; i < img2.n_rows(); ++i) {
        for (size_t j = 0; j < img2.n_cols(); ++j) {
            Mat2d<float> pt(3, 1);
            pt[0][0] = float(j) / img2.n_cols();
            pt[1][0] = float(i) / img2.n_rows();
            pt[2][0] = 1.f;
            auto transformed = homo * pt;
            Hx.at(min(max(transformed[1][0] / transformed[2][0] * img2.n_rows() - min(min_y, 0), 0.f), Hx.n_rows() - 1.f), min(max(transformed[0][0] / transformed[2][0] * img2.n_cols() - min(min_x, 0), 0.f), Hx.n_cols() - 1.f), 0)= img2.at(i, j, 0);
        }
    }
    for (size_t i = 0; i < img1.n_rows(); ++i) {
        for (size_t j = 0; j < img1.n_cols(); ++j) {
            Hx.at(i - min(min_y, 0), j - min(min_x, 0), 0) = img1.at(i, j, 0);
        }
    }
    std::cout << Hx.n_cols() << ", " << Hx.n_rows() << std::endl;
    // TODO: anti-aliasing
    imageShow(Hx);
    
    return 0;
}
