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
                result.at(i, j, c) = std::abs(img1.at(i, j, c) - img2.at(i, j, c));
            }
        }
    }
    return result;
}

void GaussianPyramid::build(const Image &img) {
    octaves_ = log2f(std::min(img.n_rows(), img.n_cols())) - 2;
    float k = powf(2, 1./s_);
    
    gaussians_.resize(octaves_);
    for (auto &o : gaussians_) {
        o.resize(s_ + 3);
    }
    dogs_.resize(octaves_);
    for (auto &o : dogs_) {
        o.resize(s_ + 3);
    }
    
    if (dbl) {
        gaussians_[0][0].image = gaussianBlur(upSample(img, 2.f), sqrtf(sigma_ * sigma_ - init_sigma_ * init_sigma_ * 4));
    } else {
         gaussians_[0][0].image = gaussianBlur(img, sqrtf(sigma_ * sigma_ - init_sigma_ * init_sigma_));
    }
    gaussians_[0][0].octave = 0;
    gaussians_[0][0].scale = sigma_;
    
    float sigma_rel[s_ + 3];
    sigma_rel[0] = sigma_;
    sigma_rel[1] = sqrtf(k*k-1) * sigma_;
    for (int i = 2; i < s_+3; i++) {
        sigma_rel[i] = sigma_rel[i-1] * k;
    }
    
    // gaussian pyramid
    for (int i = 0; i < octaves_; i++) {
        for (int j = 0; j < s_ + 3; j++) {
            if (i == 0 && j == 0) {
                continue;
            } else if (j == 0) {
                gaussians_[i][j].image = downSample(gaussians_[i-1][s_].image, 2.);
                gaussians_[i][j].octave = i;
                gaussians_[i][j].scale = powf(2.f, i) * sigma_ * powf(k, j);
            } else {
                gaussians_[i][j].image = gaussianBlur(gaussians_[i][j-1].image, sigma_rel[j]);
                gaussians_[i][j].octave = i;
                gaussians_[i][j].scale = powf(2.f, i) * sigma_ * powf(k, j);
            }
        }
    }
    
    for (int i = 0; i < octaves_; i++) {
        for (int j = 0; j < s_ + 2; j++) {
//            imageShow(diff(gaussians_[i][j+1].image, gaussians_[i][j].image));
            dogs_[i][j].image = gaussians_[i][j+1].image - gaussians_[i][j].image;
            dogs_[i][j].octave = i;
            dogs_[i][j].scale = gaussians_[i][j].scale;
        }
    }
    
//    for (int i = 0; i < octaves_; i++) {
//        for (int j = 0; i < s_ + 3; j ++) {
//            printf("Scale: %f\n", gaussians_[i][j].scale);
//            imageShow(gaussians_[i][j].image);
//        }
//    }
    
//    for (int i = 0; i < octaves_ - 4; i++) {
//        for (int j = 0; i < s_ + 2; j ++) {
////            for (int r = 0; r < dogs_[i][j].image.n_rows(); r++) {
////                for (int c = 0; c < dogs_[i][j].image.n_cols(); c++) {
////                    dogs_[i][j].image.at(r,c,0) = (dogs_[i][j].image.at(r,c,0) + 1.) / 2.;
////                }
////            }
//            printf("Scale: %f\n", dogs_[i][j].scale);
//            imageShow(dogs_[i][j].image);
//        }
//        
//    }
}


