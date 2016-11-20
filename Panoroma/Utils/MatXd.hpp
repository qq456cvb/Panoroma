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
    std::shared_ptr<int> dims_;
    int n_elems_;
    std::shared_ptr<T> raw_data_;
    
public:
    
    MatXd() {
        n_dims_ = 0;
        n_elems_ = 0;
        raw_data_ = nullptr;
    };
    MatXd(int n_dim, ...) : MatXd() {
        dims_ = std::shared_ptr<int>(new int[n_dim], std::default_delete<int[]>());
        n_dims_ = n_dim;
        std::va_list dims;
        
        n_elems_ = 1;
        va_start ( dims, n_dim );
        for (int i = 0; i < n_dim; i++) {
            dims_.get()[i] = va_arg(dims, int);
            n_elems_ *= dims_.get()[i];
        }
        va_end(dims);
        
        raw_data_ = std::shared_ptr<T>(new T[n_elems_], std::default_delete<T[]>());
        memset(raw_data_.get(), 0, n_elems_ * sizeof(T));
    }
    
    void set_all(const T& value) {
        T* ptr = raw_data_.get();
        for (int i = 0; i < n_elems(); i++) {
            *ptr = value;
            ptr++;
        }
    }
    
    int n_elems() const {
        return n_elems_;
    }
    
    int n_dims() const {
        return n_dims_;
    }
    
    T* raw_ptr() const {
        return raw_data_.get();
    }
    
    int* dims() const {
        return dims_.get();
    }


};
#endif /* MatXd_hpp */
