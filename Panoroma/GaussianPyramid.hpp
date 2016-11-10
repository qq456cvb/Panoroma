//
//  GaussianPyramid.hpp
//  Panoroma
//
//  Created by Neil on 10/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#ifndef GaussianPyramid_hpp
#define GaussianPyramid_hpp

#include <stdio.h>
#include "Mat2d.hpp"
#include "Mat3d.hpp"

struct Laplacian {
    Image image;
    float scale;
};

struct DoG {
    Image image;
    float scale;
};

class GaussianPyramid {
    int octaves_ = 4;
    int s_ = 3;
    float sigma_ = 1.6;
    float k = sqrtf(2.);
    bool upsample_ = false;
    Laplacian* laplacians_;
    DoG* dogs_;
    
public:
    void build(const Image& img);
};

#endif /* GaussianPyramid_hpp */
