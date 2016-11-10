//
//  Debugger.cpp
//  Panoroma
//
//  Created by Neil on 07/11/2016.
//  Copyright © 2016 Neil. All rights reserved.
//

#include "Debugger.hpp"

void Debugger::test(bool cond) {
    if (!cond) {
        std::cerr << "Error condition, try to print backtrace... " << std::endl;
        Debugger::print_stacktrace();
    }
}
