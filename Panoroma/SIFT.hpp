//
//  SIFT.hpp
//  Panoroma
//
//  Created by Neil on 14/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#ifndef SIFT_hpp
#define SIFT_hpp

#include <stdio.h>
#include "GaussianPyramid.hpp"
#include <vector>

#define CONTRAST_THRESHOLD 1e-2
#define EXTREMA_THRESHOLD 1e-3
#define EDGE_THRESHOLD 6.0
#define SUBPIXEL_ITER 4
#define SUBPIXEL_CONVERGE_THRESHOLD 0.5
#define ORI_HISTOGRAMS 36
#define ORI_SIGMA 1.5
#define ORI_SIGMA_RADIUS 3 * ORI_SIGMA
#define ORI_SMOOTH_PASSES 2
#define ORI_PEAK_RATIO 0.8
#define DESC_SIDE 4
#define DESC_HIST 8
#define DESC_THRESHOLD 0.2


class SIFT {
    
    
    bool edgeResponse(const KeyPoint& kp, const DoG& dog);
    void subPixelIter(int octave, int r, int c, int intvl, Mat2d<float>& jacobian, Mat2d<float>& offset);
    std::vector<KeyPoint> assignOrientation(const std::vector<KeyPoint>& kps);
    std::vector<float> histogram(const Image& img, int x, int y, int n, int radius, float sigma);
    
public:
    GaussianPyramid pyramid_;
    std::vector<KeyPoint> key_points_;
    void extract(const Image& img, Mat2d<unsigned char> &descriptors);
    
    void computeOneKp(const KeyPoint& kp, std::vector<unsigned char>& desc);
    // compute descriptors;
    void compute(const std::vector<KeyPoint> kps, Mat2d<unsigned char>& descriptors);
};

#endif /* SIFT_hpp */
