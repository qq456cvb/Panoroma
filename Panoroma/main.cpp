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
    
    
//    char cmd[128] = "convert";
//    FILE* pipe = popen(cmd, "r");
//    Assert(true);
    
    return 0;
}
