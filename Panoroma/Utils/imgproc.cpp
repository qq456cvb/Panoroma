//
//  imgproc.cpp
//  Panoroma
//
//  Created by Neil on 08/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#include "imgproc.hpp"
#include "Debugger.hpp"
#include "SIFT.hpp"

vector<float> getGaussianKernel(float sigma) {
    int double_radius = ceil(0.3 * (sigma / 2 - 1) + 0.8) * 6;
    if (double_radius % 2 == 0) double_radius++;
    vector<float> kernel(double_radius, 0.f);
    float sigma2 = 2 * sigma * sigma;
    float sum = 0;
    for (int i = 0; i < double_radius; i++) {
        kernel[i] = expf(-(i - double_radius / 2) * (i - double_radius / 2) / sigma2);
        sum += kernel[i];
    }
    std::transform(kernel.begin(), kernel.end(), kernel.begin(), [=](float v) -> float {return v / sum;});
    return kernel;
}

Image rgb2gray(const Image& mat) {
    Assert(mat.n_channels() == 3);
    Image result(mat.n_rows(), mat.n_cols(), 1);
    for (int i = 0; i < result.n_rows(); i++) {
        for (int j = 0; j < result.n_cols(); j++) {
            result.at(i, j, 0) = mat.at(i, j, 0);
        }
    }
    return result;
}

void drawLine(Image& img, Point p1, Point p2, Color color) {
    if (img.n_channels() == 1) {
        color.r = 0.299 * color.r + 0.587 * color.g + 0.114 * color.b;
    }
    Point dir = p2 - p1;
    float step = 1.f / std::max(std::fabsf(dir.x), std::fabsf(dir.y));
    for (float i = 0; i <= 1.f; i += step) {
        Point p = p1 + dir * i;
        int x = int(p.x + 0.5f);
        int y = int(p.y + 0.5f);
        if (x >= 0 && x < img.n_cols()
            && y >= 0 && y < img.n_rows()) {
            for (int c = 0; c < img.n_channels(); c++) {
                img.at(y, x, c) = color[c];
            }
        }
    }
}

void drawCircle(Image& img, Point point, Color color, int radius, bool fill) {
    // TODO: support single channel
//    Assert(img.n_channels() == 3);
    if (img.n_channels() == 1) {
        color.r = 0.299 * color.r + 0.587 * color.g + 0.114 * color.b;
    }
    float delta_angle = atan(1. / radius);
    for (float i = 0; i < 2 * PI; i += delta_angle) {
        int x = int(cosf(i) * radius + point.x + 0.5);
        int y = int(sinf(i) * radius + point.y + 0.5);
        if (x >= 0 && x < img.n_cols()
            && y >= 0 && y < img.n_rows()) {
            for (int c = 0; c < img.n_channels(); c++) {
                img.at(y, x, c) = color[c];
            }
        }
    }
}

Image gaussianBlur(const Image& mat, float sigma) {
    auto kernel = getGaussianKernel(sigma);
    int radius = (int)kernel.size() / 2;
    Image result(mat.n_rows(), mat.n_cols(), mat.n_channels()), tmp(mat.n_rows(), mat.n_cols(), mat.n_channels());
    for (int i = 0; i < mat.n_rows(); i++) {
        for (int j = 0; j < mat.n_cols(); j++) {
            for (int c = 0; c < result.n_channels(); c++) {
                float conv = 0;
                for (int m = -radius; m <= radius; m++) {
                    conv += mat.at(i, std::min(std::max(j + m, 0), mat.n_cols() - 1), c) * kernel[m+radius];
                }
                tmp.at(i, j, c) = conv;
            }
        }
    }
    
    for (int j = 0; j < mat.n_cols(); j++) {
        for (int i = 0; i < mat.n_rows(); i++) {
            for (int c = 0; c < result.n_channels(); c++) {
                float conv = 0;
                for (int m = -radius; m <= radius; m++) {
                    conv += tmp.at(std::min(std::max(i + m, 0), tmp.n_rows() - 1), j, c) * kernel[m+radius];
                }
                result.at(i, j, c) = conv;
            }
        }
    }
    return result;
}

void resize(const Image& mat, Image& out) {
    float ratio_y = mat.n_rows() / (float)out.n_rows();
    float ratio_x = mat.n_cols() / (float)out.n_cols();
    
    vector<float> src_idx_y(out.n_rows(), 0);
    vector<float> src_idx_x(out.n_cols(), 0);
    for (int i = 0; i < out.n_rows(); i++) {
        // convert to continous domain
        src_idx_y[i] = std::min(std::max((i + 0.5f) * ratio_y - 0.5f, 0.f), mat.n_rows() - 1.f);
    }

    for (int j = 0; j < out.n_cols(); j++) {
        src_idx_x[j] = std::min(std::max((j + 0.5f) * ratio_x - 0.5f, 0.f), mat.n_cols() - 1.f);
    }
    
    for (int i = 0; i < out.n_rows(); i++) {
        int idx_y = int(src_idx_y[i]);
        float interp_y = src_idx_y[i] - idx_y;
        if (idx_y >= mat.n_rows() - 1) {
            idx_y = mat.n_rows() - 2;
            idx_y = 1.f;
        }
        for (int j = 0; j < out.n_cols(); j++) {
            int idx_x = int(src_idx_x[j]);
            float interp_x = src_idx_x[j] - idx_x;
            if (idx_x >= mat.n_cols() - 1) {
                idx_x = mat.n_cols() - 2;
                interp_x = 1.f;
            }
            for (int k = 0; k < out.n_channels(); k++) {
                out.at(i, j, k) = (1.f - interp_x) * ((1.f - interp_y) * mat.at(idx_y, idx_x, k) + interp_y * mat.at(idx_y + 1, idx_x, k))
                    + interp_x * ((1.f - interp_y) * mat.at(idx_y, idx_x + 1, k) + interp_y * mat.at(idx_y + 1, idx_x + 1, k));
            }
        }
    }
}

Image downSample(const Image& mat, double ratio) {
    Image result(ceil(mat.n_rows() / ratio), ceil(mat.n_cols() / ratio), mat.n_channels());
    resize(mat, result);
    return result;
}

Image upSample(const Image& mat, double ratio) {
    Image result(ceil(mat.n_rows() * ratio), ceil(mat.n_cols() * ratio), mat.n_channels());
    resize(mat, result);
    return result;
}


Image drawMatches(const Image& img1, const Image& img2, SIFT sift1, SIFT sift2, const std::vector<int>& matches) {
    Image img(std::max(img1.n_rows(), img2.n_rows()), img1.n_cols() + img2.n_cols(), img1.n_channels());
    for (int i = 0; i < img1.n_rows(); i++) {
        for (int j = 0; j < img1.n_cols(); j++) {
            for (int k = 0; k < img1.n_channels(); k++) {
                img.at(i, j, k) = img1.at(i, j, k);
            }
        }
    }
    for (int i = 0; i < img2.n_rows(); i++) {
        for (int j = img1.n_cols(); j < img1.n_cols() + img2.n_cols(); j++) {
            for (int k = 0; k < img2.n_channels(); k++) {
                img.at(i, j, k) = img2.at(i, j - img1.n_cols(), k);
            }
        }
    }
    
    vector<Point> pts1, pts2;
    for (int i = 0; i < sift1.key_points_.size(); i++) {
        Point p = sift1.key_points_[i].p;
        p  *= powf(SCALE_INTERVAL, sift1.key_points_[i].octave);
        drawCircle(img, p, Color(1, 0, 0), 3);
        pts1.push_back(std::move(p));
    }
    for (int i = 0; i < sift2.key_points_.size(); i++) {
        Point p = sift2.key_points_[i].p;
        p  *= powf(SCALE_INTERVAL, sift2.key_points_[i].octave);
        p.x += img1.n_cols();
        drawCircle(img, p, Color(1, 0, 0), 3);
        pts2.push_back(std::move(p));
    }
    for (int i = 0; i < matches.size(); i++) {
        if (matches[i] >= 0) {
            drawLine(img, pts1[i], pts2[matches[i]]);
        }
    }
    
    return img;
}
