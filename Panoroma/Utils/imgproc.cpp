//
//  imgproc.cpp
//  Panoroma
//
//  Created by Neil on 08/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#include "imgproc.hpp"
#include "Debugger.hpp"

Mat2d<double> getGaussianKernel(float sigma) {
    int radius = int(3 * sigma + 0.5);
    Mat2d<double> kernel(2*radius+1, 2*radius+1);
    float sigma2 = 2 * powf(sigma, 2);
    for (int i = 0; i < 2 * radius + 1; i++) {
        for (int j = 0; j < 2 * radius + 1; j++) {
            kernel[i][j] = expf((-pow(abs(i-radius), 2)-pow(abs(j-radius), 2)) / sigma2);
        }
    }
    kernel.normalizeSum();
    return kernel;
}

Image rgb2gray(const Image& mat) {
    Assert(mat.n_channels() == 3);
    Image result(mat.n_rows(), mat.n_cols(), 1);
    for (int i = 0; i < result.n_rows(); i++) {
        for (int j = 0; j < result.n_cols(); j++) {
            result.at(i, j, 0) = mat.at(i, j, 0) * 0.299 + mat.at(i, j, 1) * 0.587 + mat.at(i, j, 2) * 0.114;
        }
    }
    return result;
}

Image drawCircle(const Image& img, Point point, Color color, int radius, bool fill) {
    // TODO: support single channel
    Assert(img.n_channels() == 3);
    Image result = img;
    float delta_angle = atan(1. / radius);
    for (float i = 0; i < 2 * PI; i += delta_angle) {
        int x = int(cosf(i) * radius + point.x + 0.5);
        int y = int(sinf(i) * radius + point.y + 0.5);
        if (x >= 0 && x < img.n_cols()
            && y >= 0 && y < img.n_rows()) {
            for (int c = 0; c < img.n_channels(); c++) {
                result.at(y, x, c) = color[c];
            }
        }
    }
    return result;
}

Image gaussianBlur(const Image& mat, float sigma) {
    auto kernel = getGaussianKernel(sigma);
    int radius = kernel.n_rows() / 2;
    Image result(mat.n_rows(), mat.n_cols(), mat.n_channels());
    for (int i = 0; i < mat.n_rows(); i++) {
        for (int j = 0; j < mat.n_cols(); j++) {
            for (int c = 0; c < result.n_channels(); c++) {
                double conv = 0;
                for (int m = -radius; m <= radius; m++) {
                    for (int n = -radius; n <= radius; n++) {
                        unsigned char pixel = 0;
                        if (i+m >= 0 && i+m < mat.n_rows() && j+n >= 0 && j+n < mat.n_cols()) {
                            pixel = mat.at(i+m, j+n, c);
                        }
                        conv += pixel * kernel[m+radius][n+radius];
                    }
                }
                result.at(i, j, c) = conv;
            }
        }
    }
    return result;
}

Image downSample(const Image& mat, float ratio) {
    Image result(mat.n_rows() / ratio, mat.n_cols() / ratio, mat.n_channels());
    for (int i = 0; i < result.n_rows(); i++) {
        for (int j = 0; j < result.n_cols(); j++) {
            for (int c = 0; c < result.n_channels(); c++) {
                result.at(i, j, c) = mat.at(i * ratio, j * ratio, c);
            }
        }
    }
    return result;
}

Image upSample(const Image& mat, float ratio, bool interp) {
    Image result(mat.n_rows() * ratio, mat.n_cols() * ratio, mat.n_channels());
    if (!interp) {
        for (int i = 0; i < result.n_rows(); i++) {
            for (int j = 0; j < result.n_cols(); j++) {
                for (int c = 0; c < result.n_channels(); c++) {
                    result.at(i, j, c) = mat.at(i / ratio, j / ratio, c);
                }
            }
        }
    } else {
        for (int i = 0; i < result.n_rows(); i++) {
            for (int j = 0; j < result.n_cols(); j++) {
                for (int c = 0; c < result.n_channels(); c++) {
                    int low_i = i / ratio, high_i = int(i / ratio) + 1;
                    int low_j = j / ratio, high_j = int(j / ratio) + 1;
                    if (high_i >= mat.n_rows()) {
                        high_i = low_i;
                    }
                    if (high_j >= mat.n_cols()) {
                        high_j = low_j;
                    }
                    // bilinear interpolation
                    auto ll = mat.at(low_i, low_j, c);
                    auto lh = mat.at(low_i, high_j, c);
                    auto hl = mat.at(high_i, low_j, c);
                    auto hh = mat.at(high_i, high_j, c);
                    
                    float interp_row = (i - low_i * ratio) / ratio;
                    float interp_col = (j - low_j * ratio) / ratio;
                    
                    auto l = ll * interp_col + lh * (1 - interp_col);
                    auto h = hl * interp_col + hh * (1 - interp_col);
                    result.at(i, j, c) = l * interp_row + h * (1 - interp_row);
                }
            }
        }
    }
    

    return result;
}

