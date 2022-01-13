#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include "Pose2d.h"
#include <opencv2/opencv.hpp>

class MotionModel{
public:
	MotionModel(const float& alpha1, const float& alpha2, const float& alpha3, const float& alpha4);
    
	void sampleMotionModelOdometry(const float& delta_rot1, const float& delta_trans, const float& delta_rot2, Pose2d& xt);
    
private:
    	double alpha1_, alpha2_, alpha3_, alpha4_;
    	cv::RNG rng_;
}; // class MotionModel


#endif
