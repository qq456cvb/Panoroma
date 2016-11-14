//
//  MatXd.hpp
//  Panoroma
//
//  Created by Neil on 08/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#ifndef MatXd_hpp
#define MatXd_hpp

#include <stdio.h>
#include <memory>
#include <cstdarg>

template <typename T>
class MatXd {
protected:
    int n_dims_;
    int* dims_;
    int n_elems_;
    T* raw_data_;
    
public:
    MatXd() {
        n_dims_ = 0;
        n_elems_ = 0;
        raw_data_ = nullptr;
    };
    MatXd(int n_dim, ...) : MatXd() {
        dims_ = new int[n_dim];
        n_dims_ = n_dim;
        std::va_list dims;
        
        n_elems_ = 1;
        va_start ( dims, n_dim );
        for (int i = 0; i < n_dim; i++) {
            dims_[i] = va_arg(dims, int);
            n_elems_ *= dims_[i];
        }
        va_end(dims);
        
        raw_data_ = new T[n_elems_];
        memset(raw_data_, 0, n_elems_ * sizeof(T));
    }
    ~MatXd() {
        if (raw_data_) {
            delete [] raw_data_;
            raw_data_ = nullptr;
        }
    }
    
    void set_all(const T& value) {
        T* ptr = raw_data_;
        for (int i = 0; i < n_elems(); i++) {
            *ptr = value;
            ptr++;
        }
    }
    
    int n_elems() const {
        return n_elems_;
    }
    
    T* raw_ptr() const {
        return raw_data_;
    }
    
    int* dims() const {
        return dims_;
    }
};
#endif /* MatXd_hpp */
