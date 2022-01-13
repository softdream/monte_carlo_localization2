#include "motion_model.h"

MotionModel::MotionModel ( const float& alpha1, 
			   const float& alpha2, 
			   const float& alpha3, 
			   const float& alpha4 ):
			   alpha1_( alpha1 ), 
			   alpha2_( alpha2 ), 
			   alpha3_( alpha3 ), 
			   alpha4_( alpha4 )
{
	rng_ = cv::RNG(cv::getTickCount());
}

void MotionModel::sampleMotionModelOdometry ( const float& delta_rot1, const float& delta_trans, const float& delta_rot2, Pose2d& xt )
{
	float delta_rot1_PI = delta_rot1 - M_PI;
    	float delta_rot2_PI = delta_rot2 - M_PI;
    
	Pose2d::NormAngle(delta_rot1_PI);
    	Pose2d::NormAngle(delta_rot2_PI);
    	
	float delta_rot1_noise = std::min(fabs(delta_rot1), fabs( delta_rot1_PI) );
    	float delta_rot2_noise = std::min(fabs(delta_rot2), fabs( delta_rot2_PI) );
    
    	float delta_rot1_2 = delta_rot1_noise*delta_rot1_noise;
    	float delta_rot2_2 = delta_rot2_noise * delta_rot2_noise;
    	float delta_trans_2 = delta_trans * delta_trans;
    
    	/* 采样 */
    	float delta_rot1_hat = delta_rot1 - rng_.gaussian(alpha1_ * delta_rot1_2 + alpha2_ * delta_trans_2);
    	float delta_trans_hat = delta_trans - rng_.gaussian(alpha3_ * delta_trans_2 + alpha4_ * delta_rot1_2 + alpha4_ * delta_rot2_2);
    	float delta_rot2_hat = delta_rot2 - rng_.gaussian(alpha1_ * delta_rot2_2 + alpha2_ * delta_trans_2);
    
    	xt.x_ += delta_trans_hat * cos( xt.theta_ + delta_rot1_hat );
    	xt.y_  += delta_trans_hat * sin( xt.theta_ + delta_rot1_hat );
    	xt.theta_ +=  (delta_rot1_hat + delta_rot2_hat);
    	xt.NormAngle(xt.theta_);
}
