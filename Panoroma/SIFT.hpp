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

#define CONTRAST_THRESHOLD 5e-4
#define EXTREMA_THRESHOLD 1e-3
#define EDGE_THRESHOLD 10.0
#define SUBPIXEL_ITER 4
#define SUBPIXEL_CONVERGE_THRESHOLD 0.3

class SIFT {
    GaussianPyramid pyramid_;
    std::vector<KeyPoint> key_points_;
    
    bool edgeResponse(const KeyPoint& kp, const DoG& dog);
    void subPixelIter(const KeyPoint& kp, const DoG& dog_up, const DoG& dog, const DoG& dog_down, Mat2d<float>& jacobian, Mat2d<float>& offset);
    
public:
    void extract(const Image& img);
};

#endif /* SIFT_hpp */
