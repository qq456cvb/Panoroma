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
#include <vector>
#include "Mat2d.hpp"
#include "Mat3d.hpp"
#include "imgproc.hpp"
#include "imgio.hpp"

using namespace std;
class SIFT;
struct Gaussian {
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
    float init_sigma_ = 0.5;
    bool dbl = false;
    
    // whether upsample the original image
    bool upsample_ = false;
    
    vector<vector<Gaussian> > gaussians_;
    vector<vector<DoG> > dogs_;
public:
    friend class SIFT;
    void build(const Image& img);
};

#endif /* GaussianPyramid_hpp */
