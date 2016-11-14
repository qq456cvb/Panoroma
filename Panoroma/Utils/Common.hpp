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
};

struct Color {
    unsigned char r;
    unsigned char g;
    unsigned char b;
    Color(unsigned char red,
          unsigned char green,
          unsigned char blue) {
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

#define RED Color(255, 0, 0)
#define GREEN Color(0, 255, 0)
#define BLUE Color(0, 0, 255)

#define PI 3.14159

#endif /* Common_hpp */
