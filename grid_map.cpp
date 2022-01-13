#include "grid_map.h"

namespace map{

GridMap::GridMap()
{

}

GridMap::~GridMap()
{

}

GridMap::GridMap( const int &size_x, const int &size_y, const float cell_size )
		: size_x_( size_x ), size_y_( size_y ), cell_size_( cell_size )
{

}

bool GridMap::loadMap( const std::string &file_path )
{
	cv::Mat map = cv::imread( file_path, CV_8UC1 );
	std::cout<<"starting open the bmp map file ..."<<std::endl;

	size_x_ = map.cols;
	size_y_ = map.rows;

	std::cout<<"size_x = "<<size_x_<<std::endl;
	std::cout<<"size_y = "<<size_y_<<std::endl;

	bel_data_.resize( size_x_, size_y_ );
	bel_data_.setOnes() * 0.5;

	int point_number = 0;
	for( size_t x = 0; x < size_x_; x ++ ){
		for( size_t y = 0; y < size_y_; y ++ ){
			uint8_t pixel = map.at<uint8_t>( x, y );
			//std::cout<<"pixel = "<<(int)pixel<<std::endl;
			if( pixel == 0 ){ // occupied
				point_number ++;
				bel_data_(x, y) = 1.0;
			}
			else if( pixel == 255 ){
				bel_data_(x, y) = 0.0;
			}
			else {
				bel_data_(x, y) = 0.5;
			}
		}
	}

	std::cout<<"Grid Map Occupied Number : "<<point_number<<std::endl;
	
	return true;
}

void GridMap::showMap() const
{
	cv::Mat image = cv::Mat( size_x_, size_y_, CV_8UC3, cv::Scalar::all( 125 ) );

	for( size_t x = 0; x < size_x_; x ++ ){
                for( size_t y = 0; y < size_y_; y ++ ){
			uint8_t v = 255 - (uint8_t)( bel_data_( x, y ) * 255 );
			//std::cout<<"v = "<<(int)v<<std::endl;	
			cv::Vec3b p;
			p[0] = v;
			p[1] = v;
			p[2] = v;
		
			image.at<cv::Vec3b>(x, y) = p;
		}
	}

	cv::imshow( "grid map", image );
	cv::waitKey( 0 );
}

const Eigen::Vector2i GridMap::getIdx( const float &x, const float &y )
{
	int xidx = cvFloor( x / cell_size_ );
	int yidx = cvFloor( y / cell_size_ );

	return Eigen::Vector2i( xidx, yidx );
}

const float GridMap::getGridBel( const float &x, const float &y )
{
	Eigen::Vector2i idx = getIdx( x, y );
	
	return bel_data_( idx(0), idx(1) );
}

const float GridMap::getGridBelInt( const int i, const int j )
{
	return bel_data_( i, j );
}

void GridMap::setGridBel( const float &x, const float &y, const float bel )
{
	Eigen::Vector2i idx = getIdx( x, y );
	bel_data_( idx(0), idx(1) ) = bel;
}


}
