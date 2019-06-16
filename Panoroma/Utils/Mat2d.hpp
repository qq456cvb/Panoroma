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
#include <Eigen/Core>
#include <Eigen/LU>
#include "MatXd.hpp"
#include "Debugger.hpp"

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
    
    Mat2d(const Mat2d& mat) : Mat2d(){
        *this = mat;
    }
    
    int n_elems() const {
        return n_cols_ * n_rows_;
    }
    
    int n_cols() const {
        return n_cols_;
    }
    
    int n_rows() const {
        return n_rows_;
    }
    
    Mat2d<T>& operator=(Mat2d<T>&& mat) {
        this->raw_data_ = mat.raw_data_;
        this->dims_ = mat.dims_;
        
        n_cols_ = mat.n_cols();
        n_rows_ = mat.n_rows();
        this->n_elems_ = mat.n_elems();
        this->n_dims_ = mat.n_dims();
        return *this;
    }
    
    Mat2d<T>& operator=(const Mat2d<T>& mat) {
        this->raw_data_ = mat.raw_data_;
        this->dims_ = mat.dims_;
        
        n_cols_ = mat.n_cols();
        n_rows_ = mat.n_rows();
        this->n_elems_ = mat.n_elems();
        this->n_dims_ = mat.n_dims();
        return *this;
    }
    
    Mat2d<T> clone() const {
        Mat2d<T> mat(this->n_rows_, this->n_cols_);
        mat.raw_data_.reset(new T[this->n_elems_], std::default_delete<T[]>());
        
        mat.n_elems_ = this->n_elems_;
        memcpy(mat.raw_ptr(), this->raw_ptr(), sizeof(T) * mat.n_elems());
        memcpy(mat.dims(), this->dims(), mat.n_dims() * sizeof(int));
        return mat;
    }
    
    const T* operator[](int row) const {
        return this->raw_data_.get() + row * n_cols();
    }
    
    T* operator[](int row) {
        return this->raw_data_.get() + row * n_cols();
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
    
//    Mat2d<T>& operator*=(const Mat2d<T>& mat) {
//        EigenMatrix(*this) *= EigenMatrix(mat);
//        return *this;
//    }
    
    Mat2d<T> operator*(const Mat2d<T>& mat) {
        Mat2d<T> result(this->n_rows(), mat.n_cols());
        EigenMatrix(result) =  EigenMatrix(*this) * EigenMatrix(mat);
        return result;
    }
    
    Mat2d<T> operator/(const float& ratio) {
        Mat2d<T> result(this->n_rows(), this->n_cols());
        for (int i = 0; i < n_rows_; i++) {
            for (int j = 0; j < n_cols_; j++) {
                result[i][j] /= ratio;
            }
        }
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
    
    Mat2d<T> operator-() {
        Mat2d<T> result(this->n_rows(), this->n_cols());
        for (int i = 0; i < n_rows_; i++) {
            for (int j = 0; j < n_cols_; j++) {
                result[i][j] = -(*this)[i][j];
            }
        }
        return result;
    }
    
    Mat2d<T> inverse() {
        Assert(this->n_rows() == this->n_cols());
        Mat2d<T> result(this->n_rows(), this->n_cols());
        
        Eigen::FullPivLU<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> lu(EigenMatrix(*this));
        if (!lu.isInvertible()) {
            printf("Matrix not invertible!\n");
            return result;
        }
        EigenMatrix(result) = lu.inverse().eval();
        return result;
    }
    
    Mat2d<T> transpose() {
        Mat2d<T> result(this->n_cols(), this->n_rows());
        for (int i = 0; i < n_rows_; i++) {
            for (int j = 0; j < n_cols_; j++) {
                result[j][i] = -(*this)[i][j];
            }
        }
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

#endif /* Mat2d_hpp */
