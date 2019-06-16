//
//  RANSAC.hpp
//  Panoroma
//
//  Created by Neil on 2019/6/15.
//  Copyright © 2019 Neil. All rights reserved.
//

#ifndef RANSAC_hpp
#define RANSAC_hpp

#include <stdio.h>
#include <vector>
#include "Mat2d.hpp"
#include "Common.hpp"
using namespace std;

class RANSACHomoSolver {
public:
    int n_trial = 500;
    int min_corres = 4;
    float err_thresh = 0.02;
    Mat2d<float> solve(const vector<Point> &pts1, const vector<Point> &pts2);
};

#endif /* RANSAC_hpp */
