#ifndef __GRID_MAP_H_
#define __GRID_MAP_H_

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
//#include <nav_msgs/OccupancyGrid.h>
#include <fstream>

namespace map{

class GridMap
{
public:
	GridMap();
	GridMap( const int &size_x, const int &size_y, const float cell_size );
	~GridMap();

	bool loadMap( const std::string &file_path );
	void showMap() const;
	
	const Eigen::Vector2i getIdx( const float &x, const float &y );

	const float getGridBel( const float &x, const float &y );
	const float getGridBelInt( const int i, const int j ) const;			

	void setGridBel( const float &x, const float &y, const float bel );
	
	const int getSizeX() const
	{
		return size_x_;
	}

	const int getSizeY() const
	{
		return size_y_;
	}

	const float getCellSize() const 
	{
		return cell_size_;
	}
	
private:	
	float cell_size_ = 0.1; // m
	Eigen::MatrixXd bel_data_;
	int size_x_ = 1000;
	int size_y_ = 1000;

};

}

#endif
