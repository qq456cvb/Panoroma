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

class SIFT {
    GaussianPyramid pyramid;
    
public:
    void extract(const Image& img);
};

#endif /* SIFT_hpp */
