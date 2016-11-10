//
//  main.cpp
//  Panoroma
//
//  Created by Neil on 07/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#include <iostream>
#include "Mat2d.hpp"
#include "MatXd.hpp"
#include "imgio.hpp"
#include "imgproc.hpp"
using namespace std;

int main(int argc, const char * argv[]) {
//    Mat3d<unsigned char> img(300, 200, 3);
    
    auto lena = readImage("lena.jpg");
//    Mat3d<unsigned char> lena(300, 200, 3);
//    lena.set_all(128);
//    imageShow(lena);
    lena = rgb2gray(lena);
    imageShow(lena);
    lena = gaussianBlur(lena, 3.2);
    lena = downSample(lena, 1.5);
    imageShow(lena);
    lena = upSample(lena, 2);
    imageShow(lena);
//    char cmd[128] = "convert";
//    FILE* pipe = popen(cmd, "r");
//    Assert(true);
    
    return 0;
}
