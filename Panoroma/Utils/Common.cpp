//
//  Common.cpp
//  Panoroma
//
//  Created by Neil on 11/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#include "Common.hpp"

Point operator-(const Point& p1, const Point& p2) {
    Point p;
    p.x = p1.x - p2.x;
    p.y = p1.y - p2.y;
    return p;
}

Point operator*(const Point& p, float ratio) {
    Point pp;
    pp.x = p.x * ratio;
    pp.y = p.y * ratio;
    return pp;
}


Point operator+(const Point& p1, const Point& p2) {
    Point p;
    p.x = p1.x + p2.x;
    p.y = p1.y + p2.y;
    return p;
}
