#include "measurement_model.h"

MeasurementModel::MeasurementModel( map::GridMap* map, const float& sigma, const float& rand ):map_(map), sigma_(sigma), rand_(rand)
{
	sizeX = map->getSizeX();
	sizeY = map->getSizeY();
	cellLength = map->getCellSize();
	std::cout<<"size x = "<<sizeX<<std::endl;
	std::cout<<"size y = "<<sizeY<<std::endl;
	std::cout<<"cell length = "<<cellLength<<std::endl;


    	likelihood_data_.resize( map->getSizeX(), map->getSizeY()); 
    	likelihood_data_.setZero();
    
    	/* 构造障碍物的KD-Tree */
    	pcl::PointCloud<pcl::PointXY>::Ptr cloud (new pcl::PointCloud<pcl::PointXY>);

	int count = 0;
    	for(int i = 0; i < map->getSizeX();  i ++){
        	for(int j = 0; j < map->getSizeY(); j ++){
			//std::cout<<"bel_data ("<<i<<", "<<j<<") = "<<map->getGridBelInt(i, j)<<std::endl;
            		if(map->getGridBelInt(i, j) == 1.0){ //是障碍物就加到KD数中
				count ++;
                		pcl::PointXY pt;
                		pt.x = i * cellLength;
                		pt.y = j * cellLength;
                		cloud->push_back(pt);
            		}
        	}
	}
    	kd_tree_.setInputCloud(cloud); 
	std::cout<<"set input cloud ... occupied point number: "<<count<<std::endl;    

    	/* 对每个格子计算likelihood */
    	for(float i = 0; i < map->getSizeX(); i += 1) {
        	for(float j = 0; j < map->getSizeY(); j += 1){
            		/* 计算double x, y */
            		float x = i * cellLength;
            		float y = j * cellLength;
            		float likelihood =  likelihoodFieldRangFinderModel(x, y);
            		setGridLikelihood(x, y, likelihood);
        	}	 
	}
}


const float MeasurementModel::likelihoodFieldRangFinderModel ( const float& x, const float& y )
{
	/* 找到最近的距离 */
    	pcl::PointXY search_point;
    	search_point.x = x;
    	search_point.y = y;
    	std::vector<int> k_indices;
    	std::vector<float> k_sqr_distances;
    
	int nFound =  kd_tree_.nearestKSearch(search_point, 1, k_indices, k_sqr_distances);
    	float dist = k_sqr_distances.at(0);
    
    	/* 高斯 + random */
    	return gaussion(0.0, sigma_, dist) + rand_;
}

const float MeasurementModel::gaussion ( const float& mu, const float& sigma, float x)
{
    	return (1.0 / (sqrt( 2 * M_PI ) * sigma) ) * exp( -0.5 * (x - mu) * (x - mu) / (sigma * sigma) );
}

bool MeasurementModel::getIdx( const float& x, const float& y, Eigen::Vector2i& idx )
{
    	int xidx = cvFloor( x / cellLength );
    	int yidx  = cvFloor( y /cellLength );
    
    	if((xidx < 0) || (yidx < 0) || (xidx >= sizeX) || (yidx >= sizeY))
        	return false;

    	idx << xidx , yidx;
    	return true;
}

const float MeasurementModel::getGridLikelihood ( const float& x, const float& y)
{
    	Eigen::Vector2i idx;
    	if(!getIdx(x, y, idx))
        	return rand_;
    
	return likelihood_data_(idx(0), idx(1));
}

bool MeasurementModel::setGridLikelihood ( const float& x, const float& y, const float& likelihood )
{
    	Eigen::Vector2i idx;
    	if(!getIdx(x, y, idx))
        	return false;
    	likelihood_data_(idx(0), idx(1)) = likelihood;
    	
	return true;
}

void MeasurementModel::displayLikelihoodMap()
{
	cv::Mat map(cv::Size(sizeX, sizeY), CV_64FC1, likelihood_data_.data(), cv::Mat::AUTO_STEP);

	cv::imshow("likelihood map", map);
	cv::waitKey( 0 );
}


