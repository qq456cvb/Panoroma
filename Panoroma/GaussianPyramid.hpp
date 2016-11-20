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
#include "imgproc.hpp"
#include "imgio.hpp"

class SIFT;
struct Laplacian {
    Image image;
    float scale;
    int octave;
};

struct DoG {
    Image image;
    float scale;
    int octave;
};

class GaussianPyramid {
    int octaves_ = 4;
    int s_ = 3;
    float sigma_ = 1.6;
    
    // whether upsample the original image
    bool upsample_ = false;
    
    Laplacian* laplacians_;
    DoG* dogs_;
public:
    friend class SIFT;
    void build(const Image& img);
    
    ~GaussianPyramid() {
        if (laplacians_) {
            delete [] laplacians_;
            laplacians_ = nullptr;
        }
        if (dogs_) {
            delete [] dogs_;
            dogs_ = nullptr;
        }
    }
};

#endif /* GaussianPyramid_hpp */
