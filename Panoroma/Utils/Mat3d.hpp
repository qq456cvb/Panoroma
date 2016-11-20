//
//  Mat3d.hpp
//  Panoroma
//
//  Created by Neil on 08/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#ifndef Mat3d_hpp
#define Mat3d_hpp

#include <stdio.h>
#include <algorithm>
#include "MatXd.hpp"
#include <iostream>

template<typename T>
class Mat3d : public MatXd<T> {
    int n_rows_, n_cols_, n_channels_;
    
public:
    Mat3d() {
        n_cols_ = n_rows_ = 0;
    };
    Mat3d(int n_rows, int n_cols, int n_channels) : MatXd<T>(3, n_rows, n_cols, n_channels), n_cols_(n_cols), n_rows_(n_rows), n_channels_(n_channels) {
    };
    ~Mat3d() {
    }
    Mat3d(const Mat3d& mat) : Mat3d(mat.n_rows(), mat.n_cols(), mat.n_channels()){
        memcpy(this->raw_ptr(), mat.raw_ptr(), sizeof(T) * mat.n_elems());
    }

    
    Mat3d<T>& operator=(Mat3d<T>&& mat) {
        if (this->n_elems_ != mat.n_elems()) {
            if (this->raw_data_) {
                delete [] this->raw_data_;
            }
            this->raw_data_ = new T[mat.n_elems()];
        }
        
        n_cols_ = mat.n_cols();
        n_rows_ = mat.n_rows();
        n_channels_ = mat.n_channels();
        this->n_elems_ = mat.n_elems();
        memmove(this->raw_data_, mat.raw_ptr(), sizeof(T) * mat.n_elems());
        return *this;
    }
    
    Mat3d<T>& operator=(const Mat3d<T>& mat) {
        if (this->n_elems_ != mat.n_elems()) {
            if (this->raw_data_) {
                delete [] this->raw_data_;
            }
            this->raw_data_ = new T[mat.n_elems()];
        }
        
        n_cols_ = mat.n_cols();
        n_rows_ = mat.n_rows();
        n_channels_ = mat.n_channels();
        this->n_elems_ = mat.n_elems();
        memcpy(this->raw_data_, mat.raw_ptr(), sizeof(T) * mat.n_elems());
        return *this;
    }
    
    int n_rows() const {
        return n_rows_;
    }
    
    int n_cols() const {
        return n_cols_;
    }
    
    int n_channels() const {
        return n_channels_;
    }
    
    T& at(int row, int col, int channel) {
        return *(this->raw_data_ + (row * n_cols_ + col) * n_channels_ + channel);
    }
    
    const T& at(int row, int col, int channel) const {
        return *(this->raw_data_ + (row * n_cols_ + col) * n_channels_ + channel);
    }
    
    Mat3d<T> operator-(const Mat3d<T>& mat) {
        Mat3d<T> result(mat.n_rows(), mat.n_cols(), mat.n_channels());
        for (int i = 0; i < mat.n_rows(); i++) {
            for (int j = 0; j < mat.n_cols(); j++) {
                for (int c = 0; c < mat.n_channels(); c++) {
                    result.at(i, j, c) = this->at(i, j, c) - mat.at(i, j, c);
                }
            }
        }
        return result;
    }
    
    Mat3d<T> clone() const {
        Mat3d<T> mat = *this;
        return mat;
    }
};

typedef Mat3d<float> Image;

#endif /* Mat3d_hpp */
