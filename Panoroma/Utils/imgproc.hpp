//
//  imgproc.hpp
//  Panoroma
//
//  Created by Neil on 08/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#ifndef imgproc_hpp
#define imgproc_hpp

#include <stdio.h>
#include "Mat3d.hpp"
#include "Mat2d.hpp"

Image rgb2gray(const Image& mat);

Image gaussianBlur(const Image& mat, float sigma);

Image downSample(const Image& mat, float ratio);

Image upSample(const Image& mat, float ratio, bool interp = true);
#endif /* imgproc_hpp */
