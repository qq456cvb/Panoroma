//
//  GaussianPyramid.cpp
//  Panoroma
//
//  Created by Neil on 10/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#include "GaussianPyramid.hpp"

Image diff(const Image& img1, const Image& img2) {
    Image result(img1.n_rows(), img1.n_cols(), img1.n_channels());
    for (int i = 0; i < img1.n_rows(); i++) {
        for (int j = 0; j < img1.n_cols(); j++) {
            for (int c = 0; c < img1.n_channels(); c++) {
                result.at(i, j, c) = img1.at(i, j, c) - img2.at(i, j, c) + 128;
            }
        }
    }
    return result;
}

void GaussianPyramid::build(const Image &img) {
    float k = powf(2, 1./s_);
    
    int lap_n = octaves_ * (s_ + 3);
    int dog_n = octaves_ * (s_ + 2);
    laplacians_ = new Laplacian[lap_n];
    dogs_  = new DoG[dog_n];
    
    Image unblurred = img;
    laplacians_[0].image = gaussianBlur(img, sigma_);
    laplacians_[0].octave = 1;
    laplacians_[0].scale = sigma_;
    for (int i = 0; i < octaves_; i++) {
        for (int j = 1; j < s_ + 3; j++) {
            laplacians_[i*(s_+3) + j].image = gaussianBlur(unblurred, sigma_ * powf(k, j));
            laplacians_[i*(s_+3) + j].scale = sigma_ * powf(k, j);
            dogs_[i*(s_+2) + j-1].image = diff(laplacians_[i*(s_+3) + j].image, laplacians_[i*(s_+3) + j-1].image);
            dogs_[i*(s_+2) + j-1].scale = laplacians_[i*(s_+3) + j].scale;
        }
        if (i < octaves_ - 1) {
            laplacians_[(i+1)*(s_+3)].image = downSample(laplacians_[i*(s_+3)+s_].image, 2.);
            laplacians_[(i+1)*(s_+3)].scale = sigma_;
            unblurred = downSample(unblurred, 2.);
        }
    }
//    for (int i = 0; i < lap_n; i++) {
//        imageShow(laplacians_[i].image);
//        printf("Scale: %f\n", laplacians_[i].scale);
//    }
    
    for (int i = 0; i < dog_n; i++) {
        printf("Scale: %f\n", dogs_[i].scale);
        imageShow(dogs_[i].image);
    }
}


