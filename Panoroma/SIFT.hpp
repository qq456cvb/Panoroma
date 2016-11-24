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

#define CONTRAST_THRESHOLD 7e-3
#define EXTREMA_THRESHOLD 1e-3
#define EDGE_THRESHOLD 10.0
#define SUBPIXEL_ITER 4
#define SUBPIXEL_CONVERGE_THRESHOLD 0.5
#define ORI_HISTOGRAMS 36
#define ORI_SIGMA 1.5
#define ORI_RADIUS 3 * ORI_SIGMA
#define ORI_SMOOTH_PASSES 2
#define ORI_PEAK_RATIO 0.8
#define DESC_SIDE 4
#define DESC_HIST 8

class SIFT {
    GaussianPyramid pyramid_;
    std::vector<KeyPoint> key_points_;
    
    bool edgeResponse(const KeyPoint& kp, const DoG& dog);
    void subPixelIter(const KeyPoint& kp, const DoG& dog_up, const DoG& dog, const DoG& dog_down, Mat2d<float>& jacobian, Mat2d<float>& offset);
    std::vector<KeyPoint> assignOrientation(const std::vector<KeyPoint>& kps);
    std::vector<float> histogram(const Image& img, int x, int y, int n, int radius, float sigma);
    
public:
    void extract(const Image& img);
    
    // compute descriptors;
    void compute(const std::vector<KeyPoint> kps, Mat2d<float>& descriptors);
};

#endif /* SIFT_hpp */
