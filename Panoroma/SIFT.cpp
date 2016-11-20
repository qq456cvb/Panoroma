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
                        
                        if (!edgeResponse(kp, *dog)) {
                            key_points_.push_back(kp);
                        } else {
                            printf("filter one point...\n");
                        }
                    }
                }
            }
            
        }
    }
    
    Image image = img;
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

