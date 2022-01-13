#ifndef ROBOT_H
#define ROBOT_H

#include "Pose2d.h"

class Robot{
public:
    	Robot(const float& kl, const float& kr, const float& b, const Pose2d& T_r_l):
    
	kl_(kl), kr_(kr), b_(b), T_r_l_(T_r_l)
    	{
	
	}
    
    	double kl_, kr_, b_;
    	Pose2d T_r_l_;
};//class Robot

#endif

