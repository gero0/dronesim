//
// Created by gero on 6/28/23.
//

#ifndef PLOTTING_DRONE_H
#define PLOTTING_DRONE_H

#include <cmath>
#include <algebra.h>

class Drone {
public:
    Vector3 position{0.0, 0.0, 0.0};
    Vector3 rotation{0.0, 0.0, 0.0};
    Vector3 velocity{0.0, 0.0, 0.0};
    Vector3 angular_velocity{0.0, 0.0, 0.0};
    const float arm_len = sqrt(2);
};


#endif //PLOTTING_DRONE_H
