#ifndef MEASUREMENT_MODEL
#define MEASUREMENT_MODEL

#include "grid_map.h"
#include <pcl/kdtree/kdtree_flann.h>

class MeasurementModel{
public:
	MeasurementModel(map::GridMap* map, const float& sigma, const float& rand);
    	const float likelihoodFieldRangFinderModel(const float& x, const float& y); 
    
   	bool getIdx(const float& x, const float& y, Eigen::Vector2i& idx);
    	bool setGridLikelihood(const float& x, const float& y, const float& likelihood);
    	const float getGridLikelihood ( const float& x, const float& y);

	void displayLikelihoodMap();
private:

    	const float gaussion(const float& mu, const float& sigma, float x);
    
    	map::GridMap* map_;
    	float sigma_;
    	float rand_;
    
    	pcl::KdTreeFLANN<pcl::PointXY> kd_tree_;
    
    	/* 预先计算似然域的数据 */
    	Eigen::MatrixXd likelihood_data_;
    	float cellLength;
	int sizeX;
	int sizeY;
    
}; //class MeasurementModel

#endif


