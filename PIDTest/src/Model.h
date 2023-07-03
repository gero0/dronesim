//
// Created by gero on 7/1/23.
//

#ifndef DRONESIMPROJECT_MODEL_H
#define DRONESIMPROJECT_MODEL_H


class Model {
public:
    virtual float update(float dt, float pid_output) = 0;
};


#endif //DRONESIMPROJECT_MODEL_H
