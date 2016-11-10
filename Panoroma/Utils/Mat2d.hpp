//
//  Mat2d.hpp
//  Panoroma
//
//  Created by Neil on 07/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#ifndef Mat2d_hpp
#define Mat2d_hpp

#include <stdio.h>
#include <iostream>
#include <string>
#include "Eigen/Core"
#include "MatXd.hpp"

template <typename T>
class Mat2d : public MatXd<T>{
    int n_cols_, n_rows_;
    
public:
    Mat2d() {
        n_cols_ = n_rows_ = 0;
    };
    Mat2d(int n_rows, int n_cols) : MatXd<T>(2, n_rows, n_cols), n_cols_(n_cols), n_rows_(n_rows) {
    };
    ~Mat2d() {
    }
    
    Mat2d(const Mat2d& mat) : Mat2d(mat.n_rows(), mat.n_cols()){
        memcpy(this->raw_ptr(), mat.raw_ptr(), sizeof(T) * mat.n_elem());
    }
    
    int n_elem() const {
        return n_cols_ * n_rows_;
    }
    
    int n_cols() const {
        return n_cols_;
    }
    
    int n_rows() const {
        return n_rows_;
    }
    
    Mat2d<T>& operator=(Mat2d<T>&& mat) {
        if (this->raw_data_ && this->n_elems_ != mat.n_elem()) {
            delete [] this->raw_data_;
            this->raw_data_ = new T[mat.n_elem()];
        }
        
        n_cols_ = mat.n_cols();
        n_rows_ = mat.n_rows();
        memmove(this->raw_data_, mat.raw_ptr(), sizeof(T) * mat.n_elem());
        return *this;
    }
    
    Mat2d<T>& operator=(const Mat2d<T>& mat) {
        if (this->raw_data_ && this->n_elem_ != mat.n_elem()) {
            delete [] this->raw_data_;
            this->raw_data_ = new T[mat.n_elem()];
        }
        
        n_cols_ = mat.n_cols();
        n_rows_ = mat.n_rows();
        memcpy(this->raw_data_, mat.raw_ptr(), sizeof(T) * mat.n_elem());
        return *this;
    }
    
    const T* operator[](int row) const {
        return this->raw_data_ + row * n_cols();
    }
    
    T* operator[](int row) {
        return this->raw_data_ + row * n_cols();
    }
    
    
    Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> EigenMatrix(const Mat2d& mat) {
        return Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
                                                                                                  (T*)mat.raw_ptr(), mat.n_rows(), mat.n_cols());
    }
    
    void normalize() {
        float len = 0;
        for (int i = 0; i < n_rows_; i++) {
            for (int j = 0; j < n_cols_; j++) {
                len += powf((*this)[i][j], 2);
            }
        }
        len = sqrtf(len);
        for (int i = 0; i < n_rows_; i++) {
            for (int j = 0; j < n_cols_; j++) {
                (*this)[i][j] /= len;
            }
        }
    }
    
    void normalizeSum() {
        float len = 0;
        for (int i = 0; i < n_rows_; i++) {
            for (int j = 0; j < n_cols_; j++) {
                len += (*this)[i][j];
            }
        }
        for (int i = 0; i < n_rows_; i++) {
            for (int j = 0; j < n_cols_; j++) {
                (*this)[i][j] /= len;
            }
        }
    }
    
    Mat2d<T>& operator*=(const Mat2d<T>& mat) {
        EigenMatrix(*this) *= EigenMatrix(mat);
        return *this;
    }
    
    Mat2d<T> operator*(const Mat2d<T>& mat) {
        Mat2d<T> result(mat.n_rows(), mat.n_cols());
       EigenMatrix(result) =  EigenMatrix(*this) * EigenMatrix(mat);
        return result;
    }
    
    Mat2d<T>& operator+=(const Mat2d<T>& mat) {
        EigenMatrix(*this) += EigenMatrix(mat);
        return *this;
    }
    
    Mat2d<T> operator+(const Mat2d<T>& mat) {
        Mat2d<T> result(mat.n_rows(), mat.n_cols());
        EigenMatrix(result) =  EigenMatrix(*this) + EigenMatrix(mat);
        return result;
    }
};

template<typename T>
std::ostream& operator<<(std::ostream& os, const Mat2d<T>& mat)
{
    for (int i = 0; i < mat.n_rows(); i++) {
        for (int j = 0; j < mat.n_cols(); j++) {
            os << mat[i][j];
            if (j != mat.n_cols()-1) {
                os << ' ';
            }
        }
        os << std::endl;
    }
    return os;
}

template<typename T>
std::istream& operator>>(std::istream& is, const Mat2d<T>& mat)
{
    for (int i = 0; i < mat.n_rows(); i++) {
        for (int j = 0; j < mat.n_cols(); j++) {
            is.read(&mat[i][j], sizeof(T));
            if (j != mat.n_cols()-1) {
                is.ignore();
            }
        }
        is.ignore();
    }
    return is;
}

#define Mat22d Mat<double>(2, 2)
#define Mat33d Mat<double>(3, 3)
#define Mat44d Mat<double>(4, 4)

#endif /* Mat2d_hpp */
