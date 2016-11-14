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
#include "Common.hpp"

Image rgb2gray(const Image& mat);

Image gaussianBlur(const Image& mat, float sigma);

Image downSample(const Image& mat, float ratio);

Image upSample(const Image& mat, float ratio, bool interp = true);

Image drawCircle(const Image& img, Point point, Color color = RED, int radius = 3, bool fill = false);
#endif /* imgproc_hpp */
