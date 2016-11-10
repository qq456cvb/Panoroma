//
//  imgio.hpp
//  Panoroma
//
//  Created by Neil on 08/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#ifndef imgio_hpp
#define imgio_hpp

#include <stdio.h>
#include "Mat3d.hpp"
#include "Mat2d.hpp"
#include "CImg.h"
using namespace cimg_library;


Mat3d<unsigned char> readImage(const char* file);
void imageShow(const Mat3d<unsigned char>& img);

#endif /* imgio_hpp */
