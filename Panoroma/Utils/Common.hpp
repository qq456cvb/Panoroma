//
//  Common.hpp
//  Panoroma
//
//  Created by Neil on 11/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#ifndef Common_hpp
#define Common_hpp

#include <stdio.h>
#include "Debugger.hpp"


struct Point {
    float x;
    float y;
    Point(float xx, float yy) : x(xx), y(yy) {}
    Point(const Point& p) : x(p.x), y(p.y) {}
    Point() : x(0.), y(0.) {}
    Point& operator*=(float ratio) {
        this->x *= ratio;
        this->y *= ratio;
        return *this;
    }
    friend Point operator*(const Point&, float);
    friend Point operator-(const Point&, const Point&);
    friend Point operator+(const Point&, const Point&);
};


struct KeyPoint {
    Point p;
    float response;
    // 0~2PI catison coordinate
    float orientation;
    int octave;
    // scale in octave
    float scale_oct;
    int intvl;
    float sub_intvl;
    
    KeyPoint() {}
    KeyPoint(const Point& p) : p(p), response(0.), octave(0.), scale_oct(0.), orientation(0.), intvl(0), sub_intvl(0.) {}
    KeyPoint(const KeyPoint& kp) : p(kp.p), response(kp.response), octave(kp.octave), scale_oct(kp.scale_oct), orientation(kp.orientation), intvl(kp.intvl), sub_intvl(kp.sub_intvl) {}
};

struct Color {
    float r;
    float g;
    float b;
    Color(float red,
          float green,
          float blue) {
        r = red;
        g = green;
        b = blue;
    }
    unsigned char operator[](int i) {
        Assert(i >= 0 && i <= 2);
        switch (i) {
            case 0:
                return r;
            case 1:
                return g;
            case 2:
                return b;
            default:
                Assert(false);
        }
        return r;
    }
};

#define RED Color(1., 0, 0)
#define GREEN Color(0, 1., 0)
#define BLUE Color(0, 0, 1.)

#define PI 3.1415926535

#endif /* Common_hpp */
