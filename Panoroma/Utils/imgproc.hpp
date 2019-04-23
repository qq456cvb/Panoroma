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

class SIFT;

Image rgb2gray(const Image& mat);

Image gaussianBlur(const Image& mat, float sigma);

Image downSample(const Image& mat, double ratio);

Image upSample(const Image& mat, double ratio);

void drawCircle(Image& img, Point point, Color color = Color(1, 0, 0), int radius = 3, bool fill = false);
void drawLine(Image& img, Point p1, Point p2, Color color = Color(1, 0, 0));

Image drawMatches(const Image& img1, const Image& img2, SIFT sift1, SIFT sift2, const std::vector<int>& matches);
#endif /* imgproc_hpp */
