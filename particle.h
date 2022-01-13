#ifndef PARTICLE_H
#define PARTICLE_H

#include "Pose2d.h"

class Particle{
public:
    Particle(Pose2d pose, long double weight): pose_(pose), weight_(weight){}
public:
    Pose2d pose_;
    long double weight_;
};

#endif

