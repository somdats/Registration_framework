#pragma once

#include"Levmar.h"
#include"NurbsSurface.h"

using namespace surface;

int OptimizeUsingLevenbergMarquadt(void(*lm)(double *, double *, int , int , void *), std::vector<double> &q, 
    double &error, Ray_Intersect &r_it);
