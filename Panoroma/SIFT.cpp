//
//  SIFT.cpp
//  Panoroma
//
//  Created by Neil on 14/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#include "SIFT.hpp"

void SIFT::extract(const Image &img, Mat2d<unsigned char> &descriptors) {
    pyramid_.build(img);
    
    for (int i = 0; i < pyramid_.octaves_; i++) {
        for (int j = 1; j < pyramid_.s_ + 1; j++) {
            auto dog = pyramid_.dogs_[i][j];
            // find all extremes
            for (int r = 1; r < dog.image.n_rows() - 1; r++) {
                for (int c = 1; c < dog.image.n_cols() - 1; c++) {
                    float extrema = dog.image.at(r, c, 0);
                    if (extrema < 1.25 * CONTRAST_THRESHOLD) {
                        continue;
                    }
                    bool max = true, min = true;
                    float cmp_max = extrema - EXTREMA_THRESHOLD;
                    float cmp_min = extrema + EXTREMA_THRESHOLD;
                    for (int wr = -1; wr <= 1; wr++) {
                        for (int wc = -1; wc <= 1; wc++) {
                            for (int o = -1; o <= 1; o++) {
                                if (o == 0 && wr == 0 && wc == 0) {
                                    continue;
                                }
                                if (pyramid_.dogs_[i][j+o].image.at(r+wr, c+wc, 0) >= cmp_max) {
                                    max = false;
                                }
                                if (pyramid_.dogs_[i][j+o].image.at(r+wr, c+wc, 0) <= cmp_min) {
                                    min = false;
                                }
                            }
                        }
                    }
                    
                    if (max || min) {
                        Point p(c, r);
                        KeyPoint kp(p);
                        kp.octave = dog.octave;
                        kp.intvl = j;
                        
                        Mat2d<float> offset(3, 1);
                        Mat2d<float> jacobian(3, 1);

                        bool converge = false;
                        int t = 0;
                        for (; t < SUBPIXEL_ITER; t++) {

                            subPixelIter(i, kp.p.y, kp.p.x, kp.intvl, jacobian, offset);

                            if (fabsf(offset[0][0]) < SUBPIXEL_CONVERGE_THRESHOLD
                                && fabsf(offset[1][0]) < SUBPIXEL_CONVERGE_THRESHOLD
                                && fabsf(offset[2][0]) < SUBPIXEL_CONVERGE_THRESHOLD) { // converge
                                converge = true;
                                break;
                            }

                            kp.p.x += roundf(offset[0][0]);
                            kp.p.y += roundf(offset[1][0]);
                            kp.intvl += roundf(offset[2][0]);

                            if (kp.p.x < 1 || kp.p.x >= dog.image.n_cols()
                                || kp.p.y < 1 || kp.p.y >= dog.image.n_rows()
                                || kp.intvl < 1 || kp.intvl >= pyramid_.s_ + 1) {
                                break;
                            }
                        }

                        if (!converge) { // do not converge
                            continue;
                        }
                        // delta = -H^{-1} J, by second order Taylor expansion, D = D_hat - J^T delta + 0.5 delta^T H delta, substitute delta
                        // D = D_hat - 0.5 J^T H^{-1} J = D_hat + 0.5 J^T delta
                        float true_extrema = pyramid_.dogs_[i][kp.intvl].image.at(kp.p.y, kp.p.x, 0) + 0.5 * (jacobian.transpose() * offset)[0][0];
                    
                        if (fabsf(true_extrema) > CONTRAST_THRESHOLD && !edgeResponse(kp, dog)) {
                            kp.p.x += offset[0][0];
                            kp.p.y += offset[1][0];
                            kp.sub_intvl = offset[2][0];
                            kp.scale_oct = pyramid_.sigma_ * pow(pyramid_.scale_interval_, (kp.intvl + kp.sub_intvl) / (pyramid_.s_ + 3));
                            key_points_.push_back(kp);
                        }
                    }
                }
            }
        }
    }
    std::cout << key_points_.size() << std::endl;
    // post processing, refine keypoints
    key_points_ = assignOrientation(key_points_);


    Image image = img.clone();
    for (int i = 0; i < key_points_.size(); i++) {
        Point p = key_points_[i].p;
        p  *= powf(pyramid_.scale_interval_, key_points_[i].octave);
        drawCircle(image, p, Color(1, 0, 0), 3);
        Point p_target = p;
        p_target.x += cosf(key_points_[i].orientation) * 7.f;
        p_target.y += sinf(key_points_[i].orientation) * 7.f;
        drawLine(image, p, p_target);
    }

    imageShow(image);
    compute(key_points_, descriptors);
}

bool SIFT::edgeResponse(const KeyPoint &kp, const DoG &dog) {
    // test principle curvatures ratio
    Point p = kp.p;
    float val = dog.image.at(p.y, p.x, 0);
    // d(x+1) - dx
    float dxx = dog.image.at(p.y, p.x+1, 0) + dog.image.at(p.y, p.x-1, 0) - 2 * val;
    // d(y+1) - dy
    float dyy = dog.image.at(p.y+1, p.x, 0) + dog.image.at(p.y-1, p.x, 0) - 2 * val;
    // d(dx)y
    float dxy = (dog.image.at(p.y+1, p.x+1, 0) + dog.image.at(p.y-1, p.x-1, 0) - dog.image.at(p.y+1, p.x-1, 0) - dog.image.at(p.y-1, p.x+1, 0)) / 4.;
    
    float det = dxx * dyy - dxy * dxy;
    if (det <= 0) { // saddle point
        return true;
    }
    float trace = dxx + dyy;
    float trace2 = trace * trace;
    if (trace2 / det < (EDGE_THRESHOLD + 1) * (EDGE_THRESHOLD + 1) / EDGE_THRESHOLD) {
        return false;
    }
    return true;
}

void SIFT::subPixelIter(int octave, int r, int c, int intvl, Mat2d<float>& jacobian, Mat2d<float>& offset) {
    // d(x+1) - dx
    float dxx = pyramid_.dogs_[octave][intvl].image.at(r, c+1, 0) + pyramid_.dogs_[octave][intvl].image.at(r, c-1, 0) - 2 * pyramid_.dogs_[octave][intvl].image.at(r, c, 0);
    // d(y+1) - dy
    float dyy = pyramid_.dogs_[octave][intvl].image.at(r+1, c, 0) + pyramid_.dogs_[octave][intvl].image.at(r-1, c, 0) - 2 * pyramid_.dogs_[octave][intvl].image.at(r, c, 0);
    // d(dx)y
    float dxy = (pyramid_.dogs_[octave][intvl].image.at(r+1, c+1, 0) + pyramid_.dogs_[octave][intvl].image.at(r-1, c-1, 0) - pyramid_.dogs_[octave][intvl].image.at(r+1, c-1, 0) - pyramid_.dogs_[octave][intvl].image.at(r-1, c+1, 0)) / 4.;
    
    float dxsigma = ((pyramid_.dogs_[octave][intvl+1].image.at(r, c+1, 0) - pyramid_.dogs_[octave][intvl-1].image.at(r, c+1, 0)) / 2 - (pyramid_.dogs_[octave][intvl+1].image.at(r, c-1, 0) - pyramid_.dogs_[octave][intvl-1].image.at(r, c-1, 0)) / 2) / 2;
    
     float dysigma = ((pyramid_.dogs_[octave][intvl+1].image.at(r+1, c, 0) - pyramid_.dogs_[octave][intvl-1].image.at(r+1, c, 0)) / 2 - (pyramid_.dogs_[octave][intvl+1].image.at(r-1, c, 0) - pyramid_.dogs_[octave][intvl-1].image.at(r-1, c, 0)) / 2) / 2;
    
    float dsigmasigma = pyramid_.dogs_[octave][intvl+1].image.at(r, c, 0) + pyramid_.dogs_[octave][intvl-1].image.at(r, c, 0) - pyramid_.dogs_[octave][intvl].image.at(r, c, 0) * 2;
    
    Mat2d<float> hessian(3, 3);
    hessian[0][0] = dxx;
    hessian[0][1] = hessian[1][0] = dxy;
    hessian[0][2] = hessian[2][0] = dxsigma;
    hessian[1][1] = dyy;
    hessian[2][1] = hessian[1][2] = dysigma;
    hessian[2][2] = dsigmasigma;
    
//    Mat2d<float> jacobian(3, 1);
    jacobian[0][0] = (pyramid_.dogs_[octave][intvl].image.at(r, c+1, 0) - pyramid_.dogs_[octave][intvl].image.at(r, c-1, 0)) / 2;
    jacobian[1][0] = (pyramid_.dogs_[octave][intvl].image.at(r+1, c, 0) - pyramid_.dogs_[octave][intvl].image.at(r-1, c, 0)) / 2;
    jacobian[2][0] = (pyramid_.dogs_[octave][intvl+1].image.at(r, c, 0) - pyramid_.dogs_[octave][intvl-1].image.at(r, c, 0)) / 2;
    
    offset = -hessian.inverse() * jacobian;
//    std::cout << offset << std::endl;
    
}

void smoothHist(std::vector<float>& hist) {
    std::vector<float> prev = hist;
    for (int i = 0; i < hist.size(); i++) {
        hist[i] = prev[i-1<0?(hist.size()-1):(i-1)] * 0.25 + prev[i] * 0.5 + prev[i+1>=hist.size()?0:(i+1)] * 0.25;
    }
}

void addGoodOriKeypoints(const std::vector<float>& hist, const float threshold, std::vector<KeyPoint>& kps, KeyPoint ref_kp) {
    for (int i = 0; i < ORI_HISTOGRAMS; i++) {
        int l = i-1<0?0:(i-1);
        int r = i+1>= ORI_HISTOGRAMS?(ORI_HISTOGRAMS-1):(i+1);
        
        if (hist[i] > hist[l] && hist[i] > hist[r] && hist[i] > threshold) {
            // find one
            // newton method interpolation
            float di = (hist[l] - hist[r]) / 2.;
            float didi = hist[l] + hist[r] - 2 * hist[i];
            float offset = di / didi;
            float true_i = i + offset;
            if (true_i < 0) true_i += ORI_HISTOGRAMS;
            if (true_i >= ORI_HISTOGRAMS) true_i -= ORI_HISTOGRAMS;
            float true_ori = true_i / ORI_HISTOGRAMS * 2 * PI;
            
            // add it to keypoints
            KeyPoint kp = ref_kp;
            kp.orientation = true_ori;
            kps.push_back(kp);
        }
    }
}


std::vector<KeyPoint> SIFT::assignOrientation(const std::vector<KeyPoint> &kps) {
    std::vector<KeyPoint> results;
    for (auto& kp : kps) {
        int round_x = roundf(kp.p.x);
        int round_y = roundf(kp.p.y);
        
        auto hist = histogram(pyramid_.gaussians_[kp.octave][kp.intvl].image, round_x, round_y, ORI_HISTOGRAMS, roundf(ORI_SIGMA_RADIUS * kp.scale_oct), ORI_SIGMA* kp.scale_oct);
        
        for (int i = 0; i < ORI_SMOOTH_PASSES; i++) {
            smoothHist(hist);
        }
        
        float max_mag = *std::max_element(hist.begin(), hist.end());
        addGoodOriKeypoints(hist, max_mag * ORI_PEAK_RATIO, results, kp);
    }
    
    return results;
}

std::vector<float> SIFT::histogram(const Image& img, int x, int y, int n, int radius, float sigma) {
    std::vector<float> hist;
    hist.resize(n);
    
    // gaussian weights distribution
    for (int i = -radius; i < radius; i++) {
        for (int j = -radius; j < radius; j++) {
            if (x+i < 1 || x+i >= img.n_cols() - 1
                || y+j < 1 || y +j >= img.n_rows() - 1) {
                continue;
            }
            if (i * i + j * j > radius * radius) continue;
            
            float dx = img.at(y+j, x+i+1, 0) - img.at(y+j, x+i-1, 0);
            float dy = img.at(y+j+1, x+i, 0) - img.at(y+j-1, x+i, 0);
            float mag = sqrtf(powf(dx, 2.)
                              + powf(dy, 2));
            float ori = atan2f(dy, dx) + PI;
            
            float weight = expf(-(i*i+j*j) / (2*sigma*sigma));
            int bin = roundf(n * ori / (2*PI));
            if (bin >= n) {
                bin = 0;
            }
            hist[bin] = weight * mag;
        }
    }
    
    return hist;
}

void normalize(std::vector<std::vector<std::vector<float>>>& hist) {
    float len = 0.;
    for (int i = 0; i < DESC_SIDE; i++) {
        for (int j = 0; j < DESC_SIDE; j++) {
            for (int k = 0; k < DESC_HIST; k++) {
                len += hist[i][j][k] * hist[i][j][k];
            }
        }
    }
    len = sqrtf(len);
    for (int i = 0; i < DESC_SIDE; i++) {
        for (int j = 0; j < DESC_SIDE; j++) {
            for (int k = 0; k < DESC_HIST; k++) {
                hist[i][j][k] /= len;
            }
        }
    }
}

void interpHist(std::vector<std::vector<std::vector<float>>>& hist, float x, float y, float o, float mag) {
    float d_c = x - floorf(x);
    float d_r = y - floorf(y);
    float d_o = o - floorf(o);
    
    for (int r = int(floorf(y)), rc = 0; r <= int(floorf(y)) + 1; r++, rc++) {
        if (r < 0 || r >= DESC_SIDE) {
            continue;
        }
        for (int c = int(floorf(x)), cc = 0; c <= int(floorf(x)) + 1; c++, cc++) {
            if (c < 0 || c >= DESC_SIDE) {
                continue;
            }
            for (int k = int(floorf(o)), oc = 0; k <= int(floorf(o)) + 1; k++, oc++) {
                int ori = k;
                if (k >= DESC_HIST) {
                    ori = DESC_HIST - 1;
                }
                // every position is affected by its (3d) two adjacent values
                hist[r][c][ori] += mag * powf(d_r, rc) * powf(1-d_r, 1-rc)
                                                * powf(d_c, cc) * powf(1-d_c, 1-cc)
                                                * powf(d_o, oc) * powf(1-d_o, 1-oc);
            }
        }
    }
}

void SIFT::computeOneKp(const KeyPoint& kp, std::vector<unsigned char>& desc) {
    std::vector<std::vector<std::vector<float>>> hist;
    hist.resize(DESC_SIDE);
    for (auto& i:hist) {
        i.resize(DESC_SIDE);
        for (auto& k:i) {
            k.resize(DESC_HIST);
            for (auto& num:k) {
                num = 0.;
            }
        }
    }
    
    float hist_diameter = 3 * kp.scale_oct;
    // add one more desc side for interpolation
    // add 0.5 to ceil
    int radius = hist_diameter * sqrtf(2.) * (DESC_SIDE + 1) / 2. + 0.5;
    
    float ori = kp.orientation;
    float cos = cosf(ori);
    float sin = sinf(ori);
    for (int x = -radius; x <= radius; x++) {
        for (int y = -radius; y <= radius; y++) {
            // rotate base
            float x_rot = (cos * x + sin * y) / hist_diameter;
            float y_rot = (-sin * x + cos * y) / hist_diameter;
            
            float x_bin = x_rot + DESC_SIDE / 2 - 0.5;
            float y_bin = y_rot + DESC_SIDE / 2 - 0.5;
            
            if (x_bin > -1 && x_bin < DESC_SIDE
                && y_bin > -1 && y_bin < DESC_SIDE) {
                Image img = pyramid_.gaussians_[kp.octave][kp.intvl].image;
                
                int c = int(roundf(kp.p.x)) + x;
                int r = int(roundf(kp.p.y)) + y;
                if (r < 1 || r >= img.n_rows() - 1
                    || c < 1 || c >= img.n_cols() - 1) {
                    continue;
                }
                
                float dx = img.at(r, c+1, 0) - img.at(r, c-1, 0);
                float dy = img.at(r+1, c, 0) - img.at(r-1, c, 0);
                float mag = sqrtf(powf(dx, 2.)
                                  + powf(dy, 2));
                float grad_ori = atan2f(dy, dx);
                
                grad_ori -= ori;
                while (grad_ori < 0) {
                    grad_ori += 2 * PI;
                }
                while (grad_ori >= 2 * PI) {
                    grad_ori -= 2 * PI;
                }
                
                float o_bin = grad_ori / (2*PI) * DESC_HIST;
                float subregion_weight = expf(-(x_rot * x_rot + y_rot * y_rot) / (2 * (powf(0.5 * DESC_SIDE, 2.))));
                interpHist(hist, x_bin, y_bin, o_bin, subregion_weight * mag);
            }
        }
    }
    

    normalize(hist);
    for (int i = 0; i < DESC_SIDE; i++) {
        for (int j = 0; j < DESC_SIDE; j++) {
            for (int k = 0; k < DESC_HIST; k++) {
                if (hist[i][j][k] > DESC_THRESHOLD) {
                    hist[i][j][k] = DESC_THRESHOLD;
                }
            }
        }
    }
    normalize(hist);

    desc.resize(DESC_SIDE * DESC_SIDE * DESC_HIST);
    for (int i = 0; i < DESC_SIDE; i++) {
        for (int j = 0; j < DESC_SIDE; j++) {
            for (int k = 0; k < DESC_HIST; k++) {
                // due to the suggestion by Lowe, scale by 512
                desc[i * DESC_SIDE * DESC_HIST + j * DESC_HIST + k] = std::min((int)(hist[i][j][k] * 512 + 0.5), 255);
            }
        }
    }
}

void SIFT::compute(const std::vector<KeyPoint> kps, Mat2d<unsigned char> &descriptors) {
    
    descriptors = Mat2d<unsigned char>(static_cast<int>(kps.size()), DESC_SIDE * DESC_SIDE * DESC_HIST);
    
    for (int i = 0; i < kps.size(); i++) {
        std::vector<unsigned char> desc;
        computeOneKp(kps[i], desc);
        
        for (int j = 0; j < desc.size(); j++) {
            descriptors[i][j] = desc[j];
        }
    }
    
}






