#ifndef LOCALIZER_H
#define LOCALIZER_H

#include "robot.h"
#include "grid_map.h"
#include "measurement_model.h"
#include "motion_model.h"
#include "particle.h"

#include "data_type.h"


class Localizer{
public:
    	Localizer(Robot* robot, map::GridMap* map, MotionModel* motion_model, MeasurementModel* measurement_model, size_t nParticles);
    
	void motionUpdate(const Pose2d& odom);
    	
	void measurementUpdate(const sensor::LaserScan& scan );
    	
	void reSample();
    
	//void particles2RosPoseArray(geometry_msgs::PoseArray& pose_array);
    	void normalization();
 
	void displayMap() const;
	void displayMapWithScan( const Pose2d &pose, const sensor::LaserScan &scan ) const;   	

private:
    	Robot* robot_;
    	map::GridMap* map_;
    	MotionModel* motion_model_;
    	MeasurementModel* measurement_model_;
    
    	size_t nParticles_;
    	std::vector<Particle> particles_; //粒子
    
    	/* sys statues */
    	bool is_init_;
    	Pose2d last_odom_pose_;
}; //class Localizer

#endif

