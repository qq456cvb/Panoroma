//
//  SIFT.cpp
//  Panoroma
//
//  Created by Neil on 14/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#include "SIFT.hpp"

void SIFT::extract(const Image &img) {
    pyramid_.build(img);
    
    for (int i = 0; i < pyramid_.octaves_; i++) {
        for (int j = 1; j < pyramid_.s_ + 1; j++) {
//            printf("processing extrema at: %d, %d\n", i, j);
            auto dog_up = &pyramid_.dogs_[i * (pyramid_.s_ + 2) + j + 1];
            auto dog = &pyramid_.dogs_[i * (pyramid_.s_ + 2) + j];
            auto dog_down = &pyramid_.dogs_[i * (pyramid_.s_ + 2) + j - 1];
            
            std::vector<DoG*> set = {dog_down, dog, dog_up};
            // find all extremes
            for (int r = 1; r < dog->image.n_rows() - 1; r++) {
                for (int c = 1; c < dog->image.n_cols() - 1; c++) {
                    float extrema = dog->image.at(r, c, 0);
                    bool max = true, min = true;
                    float cmp_max = extrema - EXTREMA_THRESHOLD;
                    float cmp_min = extrema + EXTREMA_THRESHOLD;
                    for (int wr = -1; wr <= 1; wr++) {
                        for (int wc = -1; wc <= 1; wc++) {
                            for (int o = -1; o <= 1; o++) {
                                if (o == 0 && wr == 0 && wc == 0) {
                                    continue;
                                }
                                if (set[1+o]->image.at(r+wr, c+wc, 0) > cmp_max) {
                                    max = false;
                                }
                                if (set[1+o]->image.at(r+wr, c+wc, 0) < cmp_min) {
                                    min = false;
                                }
                            }
                        }
                    }
                    
                    if (max || min) {
                        // add keypoint
                        Point p(c, r);
                        KeyPoint kp(p);
                        kp.octave = dog->octave;
                        kp.scale = dog->scale;
                        kp.scale_id = j;
                        
                        
                        if (!edgeResponse(kp, *dog)) {
                            Mat2d<float> offset(3, 1);
                            Mat2d<float> jacobian(3, 1);
                            auto dog_iter_up = dog_up;
                            auto dog_iter = dog;
                            auto dog_iter_down = dog_down;
                            
                            bool converge = false;
                            int t = 0;
                            for (; t < SUBPIXEL_ITER; t++) {
                                if (kp.p.x < 1 || kp.p.x >= dog_iter->image.n_cols()
                                    || kp.p.y < 1 || kp.p.y >= dog_iter->image.n_rows()
                                    || kp.scale_id < 1 || kp.scale_id >= pyramid_.s_ + 1) {
                                    break;
                                }
                                
                                subPixelIter(kp, *dog_iter_up, *dog_iter, *dog_iter_down, jacobian, offset);
                                
                                if (fabsf(offset[0][0]) < SUBPIXEL_CONVERGE_THRESHOLD
                                    && fabsf(offset[1][0]) < SUBPIXEL_CONVERGE_THRESHOLD
                                    && fabsf(offset[2][0]) < SUBPIXEL_CONVERGE_THRESHOLD) { // converge
                                    converge = true;
                                    break;
                                }
                                
                                kp.p.x += roundf(offset[0][0]);
                                kp.p.y += roundf(offset[1][0]);
                                kp.scale_id += roundf(offset[2][0]);
                                if (std::abs(roundf(offset[2][0]) - 1.) < 0.01) {
                                    dog_iter_up++;
                                    dog_iter++;
                                    dog_iter_down++;
                                } else if (std::abs(roundf(offset[2][0]) + 1.) < 0.01) {
                                    dog_iter_up--;
                                    dog_iter--;
                                    dog_iter_down--;
                                }
                            }
                            
                            float true_extrema = dog_iter->image.at(kp.p.y, kp.p.x, 0) + (jacobian.transpose() * offset)[0][0] / 2;
                            if (!converge) { // do not converge
                                continue;
                            }
                            if (true_extrema > CONTRAST_THRESHOLD / pyramid_.s_) {
                                float k = powf(2, 1./pyramid_.s_);
                                kp.p.x += offset[0][0];
                                kp.p.y += offset[1][0];
                                kp.scale_id += offset[2][0];
                                kp.scale = pyramid_.sigma_ * powf(k, kp.scale_id);
                                key_points_.push_back(kp);
                            }
                        }
                    }
                }
            }
            
        }
    }
    
    // post processing, refine keypoints
    key_points_ = assignOrientation(key_points_);
    
    
    Image image = img.clone();
    for (int i = 0; i < key_points_.size(); i++) {
        Point p = key_points_[i].p;
        p  = p * powf(2, key_points_[i].octave);
        drawCircle(image, p);
    }
    
    imageShow(image);
}

bool SIFT::edgeResponse(const KeyPoint &kp, const DoG &dog) {
    // test principle curvatures ratio
    Point p = kp.p;
    // d(x+1) - dx
    float dxx = dog.image.at(p.y, p.x+1, 0) + dog.image.at(p.y, p.x-1, 0) - 2 * dog.image.at(p.y, p.x, 0);
    // d(y+1) - dy
    float dyy = dog.image.at(p.y+1, p.x, 0) + dog.image.at(p.y-1, p.x, 0) - 2 * dog.image.at(p.y, p.x, 0);
    // d(dx)y
    float dxy = (dog.image.at(p.y+1, p.x+1, 0) + dog.image.at(p.y-1, p.x-1, 0) - dog.image.at(p.y+1, p.x-1, 0) - dog.image.at(p.y-1, p.x+1, 0)) / 4.;
    
    float det = dxx * dyy - dxy * dxy;
    if (det < 0) { // saddle point
        return true;
    }
    float trace = dxx + dyy;
    if (trace * trace / det < (EDGE_THRESHOLD + 1) * (EDGE_THRESHOLD + 1) / EDGE_THRESHOLD) {
        return false;
    }
    return true;
}

void SIFT::subPixelIter(const KeyPoint& kp, const DoG& dog_up, const DoG& dog, const DoG& dog_down, Mat2d<float>& jacobian, Mat2d<float>& offset) {
    
    Point p = kp.p;
    // d(x+1) - dx
    float dxx = dog.image.at(p.y, p.x+1, 0) + dog.image.at(p.y, p.x-1, 0) - 2 * dog.image.at(p.y, p.x, 0);
    // d(y+1) - dy
    float dyy = dog.image.at(p.y+1, p.x, 0) + dog.image.at(p.y-1, p.x, 0) - 2 * dog.image.at(p.y, p.x, 0);
    // d(dx)y
    float dxy = (dog.image.at(p.y+1, p.x+1, 0) + dog.image.at(p.y-1, p.x-1, 0) - dog.image.at(p.y+1, p.x-1, 0) - dog.image.at(p.y-1, p.x+1, 0)) / 4.;
    
    float dxsigma = ((dog_up.image.at(p.y, p.x+1, 0) - dog_down.image.at(p.y, p.x+1, 0)) / 2 - (dog_up.image.at(p.y, p.x-1, 0) - dog_down.image.at(p.y, p.x-1, 0)) / 2) / 2;
    
     float dysigma = ((dog_up.image.at(p.y+1, p.x, 0) - dog_down.image.at(p.y+1, p.x, 0)) / 2 - (dog_up.image.at(p.y-1, p.x, 0) - dog_down.image.at(p.y-1, p.x, 0)) / 2) / 2;
    
    float dsigmasigma = dog_up.image.at(p.y, p.x, 0) + dog_down.image.at(p.y, p.x, 0) - dog.image.at(p.y, p.x, 0) * 2;
    
    Mat2d<float> hessian(3, 3);
    hessian[0][0] = dxx;
    hessian[0][1] = hessian[1][0] = dxy;
    hessian[0][2] = hessian[2][0] = dxsigma;
    hessian[1][1] = dyy;
    hessian[2][1] = hessian[1][2] = dysigma;
    hessian[2][2] = dsigmasigma;
    
//    Mat2d<float> jacobian(3, 1);
    jacobian[0][0] = (dog.image.at(p.y, p.x+1, 0) - dog.image.at(p.y, p.x-1, 0)) / 2;
    jacobian[1][0] = (dog.image.at(p.y+1, p.x, 0) - dog.image.at(p.y-1, p.x, 0)) / 2;
    jacobian[2][0] = (dog_up.image.at(p.y, p.x, 0) - dog_down.image.at(p.y, p.x, 0)) / 2;
    
    offset = - hessian.inverse() * jacobian;
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
        int round_scale_id = roundf(kp.scale_id);
        
        auto hist = histogram(pyramid_.laplacians_[kp.octave * (pyramid_.s_ + 3) + round_scale_id].image, round_x, round_y, ORI_HISTOGRAMS, ORI_RADIUS * kp.scale, ORI_SIGMA* kp.scale);
        
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
    
    // not strictly circle
    // gaussian weights distribution
    for (int i = -radius; i <= radius; i++) {
        for (int j = -radius; j <= radius; j++) {
            if (x+i < 0 || x+i >= img.n_cols()
                || y+j < 0 || y +j >= img.n_rows()) {
                continue;
            }
            
            float dx = img.at(y+j, x+i+1, 0) - img.at(y+j, x+i-1, 0);
            float dy = img.at(y+j+1, x+i, 0) - img.at(y+j-1, x+i, 0);
            float mag = sqrtf(powf(dx, 2.)
                              + powf(dy, 2));
            float ori = atan2f(dy, dx);
            // basis is inverted
            if (ori < 0) { // 0-2PI
                ori += 2*PI;
            }
            
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
                hist[r][c][k] += mag * powf(d_r, rc) * powf(1-d_r, 1-rc)
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
    
    float hist_diameter = 3 * kp.scale;
    // add one more desc side for interpolation
    // add 0.5 to ceil
    int radius = hist_diameter * sqrtf(2.) * (DESC_SIDE + 1) / 2. + 0.5;
    
    float ori = kp.orientation;
    float cos = cosf(ori);
    float sin = sinf(ori);
    for (int x = -radius; x <= radius; x++) {
        for (int y = -radius; y <= radius; y++) {
            // rotating basis equals to ratote the point in reverse direction
            float x_rot = (cos * x + sin * y) / hist_diameter;
            float y_rot = (-sin * x + cos * y) / hist_diameter;
            
            float x_bin = x_rot + DESC_SIDE / 2 - 0.5;
            float y_bin = y_rot + DESC_SIDE / 2 - 0.5;
            
            if (x_bin > -1 && x_bin < DESC_SIDE
                && y_bin > -1 && y_bin < DESC_SIDE) {
                Image img = pyramid_.laplacians_[kp.octave * (pyramid_.s_ + 3) + int(roundf(kp.scale_id))].image;
                
                int c = int(roundf(kp.p.x)) + x;
                int r = int(roundf(kp.p.y)) + y;
                if (r < 0 || r >= img.n_cols()
                    || c < 0 || c >= img.n_rows()) {
                    continue;
                }
                
                float dx = img.at(r, c+1, 0) - img.at(r, c-1, 0);
                float dy = img.at(r+1, c, 0) - img.at(r-1, c, 0);
                float mag = sqrtf(powf(dx, 2.)
                                  + powf(dy, 2));
                float ori = atan2f(dy, dx);
                // basis is inverted
                if (ori < 0) { // 0-2PI
                    ori += 2*PI;
                }
                
                float o_bin = ori / (2*PI) * DESC_HIST;
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






