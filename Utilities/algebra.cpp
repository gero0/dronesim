//
// Created by gero on 7/8/23.
//

#include "algebra.h"

float normalize_angle( float angle )
{
    const float full = 2.0f * M_PI;
    angle = fmodf(angle, full);
    if (angle > M_PI)
    {
        angle -= full;
    }
    if (angle < -M_PI)
    {
        angle += full;
    }
    return angle;
}