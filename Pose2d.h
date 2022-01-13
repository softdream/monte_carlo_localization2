#ifndef POSE2D_H
#define POSE2D_H

#include <eigen3/Eigen/Core>
#include <math.h>

class Pose2d{
public:
	Pose2d()
	{

    	}
	
    	Pose2d(const float x, const float y, const float theta):x_(x), y_(y), theta_(theta)
	{

	}
    
    	const Pose2d operator*(const Pose2d& p2)   
    	{  
        	Pose2d p;
        	Eigen::Matrix2d R;
        	R << cos(theta_), -sin(theta_),

        	sin(theta_), cos(theta_);
        	Eigen::Vector2d pt2(p2.x_, p2.y_);          
        	Eigen::Vector2d pt = R * pt2 + Eigen::Vector2d(x_, y_);
        
        	p.x_ = pt(0);
        	p.y_ = pt(1);
        	p.theta_ = theta_ + p2.theta_;
        
		NormAngle( p.theta_);
        
		return p;
    	}  
    
    	const Eigen::Vector2f operator*(const Eigen::Vector2f& p)   
    	{  
        	Eigen::Matrix2f R;

        	R << cos(theta_), -sin(theta_),
        	     sin(theta_), cos(theta_);
        	
		Eigen::Vector2f t(x_, y_);
        	
		return R * p + t;
    	}  
    
    
    	const Pose2d inv()
    	{
        	float x = -( cos(theta_) * x_ + sin(theta_) * y_);
        	float y = -( -sin(theta_) * x_ + cos(theta_) * y_);
        	float theta = -theta_;
        	
		return Pose2d(x, y, theta);
    	}
    
    	static void NormAngle ( float& angle )
    	{        
		if( angle >= M_PI)
            		angle -= 2.0f * M_PI;

        	if( angle < -M_PI)
            		angle += 2.0f * M_PI;
    	}
	
    	float x_ = 0.0f, y_ = 0.0f , theta_ = 0.0f; 
}; //class Pose2d


#endif

