#include "grid_map.h"
#include "measurement_model.h"
#include "motion_model.h"
#include "robot.h"
#include "particle.h"
#include "localizer.h"

#include "data_read.h"

int main()
{
	std::cout<<"----------------- Monte Carlo Test -----------------"<<std::endl;
	// ----------------- 1. init the grid map ------------------- //
	map::GridMap *grid_map = new map::GridMap;
	grid_map->loadMap( "./test.bmp" );
	
	grid_map->showMap();

	// ------------- 2. init the measurement model-------------- //
	float sigma = 0.2f;
	float rand = 0.01f;
	std::cout<<"caculate the likelihood now ...."<<std::endl;
        MeasurementModel *measurement_model = new MeasurementModel( grid_map, sigma, rand );
        std::cout<<"likelihood map caculated finished ..."<<std::endl;
	measurement_model->displayLikelihoodMap();

	// --------------- 3. init the motion modle --------------- //
	float a1 = 0.15, a2 = 0.15, a3 = 0.15, a4 = 0.15;
	MotionModel *motion_model = new MotionModel( a1, a2, a3, a4 );
	
	// ---------------- 4. init the robot paramters ----------- //
	Pose2d T_r_l( 0, 0, 0 );
	Robot *robot = new Robot( 0.0f, 0.0f, 0.0f, T_r_l );

	/*for( int i = 0; i < 1000; i ++ ){
		for( int j = 0; j < 1000; j ++ ){
			std::cout<<"map["<<i<<", "<<j<<"]: "<<grid_map->getGridBelInt( i, j )<<std::endl;
			cv::waitKey(10);
		}
	}*/

	// ----------------- 5. init the localizer --------------- //
	int particles = 1000;
	Localizer *localizer = new Localizer( robot, grid_map, motion_model, measurement_model, particles );
	localizer->displayMap();

	cv::waitKey( 0 );
	return 0;
}
