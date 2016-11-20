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

#define EXTREMA_THRESHOLD 1e-3
#define EDGE_THRESHOLD 10

class SIFT {
    GaussianPyramid pyramid_;
    std::vector<KeyPoint> key_points_;
    
    bool edgeResponse(const KeyPoint& kp, const DoG& dog);
    
public:
    void extract(const Image& img);
};

#endif /* SIFT_hpp */
