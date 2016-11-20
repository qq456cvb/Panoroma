//
//  imgio.cpp
//  Panoroma
//
//  Created by Neil on 08/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#include "imgio.hpp"
#include "Debugger.hpp"

Image readImage(const char* file) {
    CImg<unsigned char> image(file);
    Assert(image.spectrum() == 3 || image.spectrum() == 1);
    Image result(image.height(), image.width(), image.spectrum());
    cimg_forXYC(image, x, y, c) {
        result.at(y, x, c) = ((float)image(x, y, c)) / 255.;
    }
    return result;
}

void imageShow(const Image& img) {
    CImg<unsigned char> image(img.n_cols(), img.n_rows(), 1, img.n_channels());
    cimg_forXYC(image, x, y, c) {
        image(x, y, c) = img.at(y, x, c) * 255;
    }
    CImgDisplay main_disp(image,"Image");
    while (!main_disp.is_closed()) {
        main_disp.wait();
    }
}
