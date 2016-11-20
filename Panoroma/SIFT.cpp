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
                            
                            float true_extrema = dog->image.at(kp.p.y, kp.p.x, 0) + (jacobian.transpose() * offset)[0][0] / 2;
                            if (!converge) { // do not converge
                                continue;
                            }
                            if (true_extrema > CONTRAST_THRESHOLD) {
                                float k = powf(2, 1./pyramid_.s_);
                                kp.p.x += offset[0][0];
                                kp.p.y += offset[1][0];
                                kp.scale = pyramid_.sigma_ * powf(k, kp.scale_id);
                                key_points_.push_back(kp);
                            }
                        }
                    }
                }
            }
            
        }
    }
    
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


