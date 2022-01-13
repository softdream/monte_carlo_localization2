#include "grid_map.h"
#include "measurement_model.h"


int main()
{
	std::cout<<"----------------- Monte Carlo Test -----------------"<<std::endl;
	map::GridMap *grid_map = new map::GridMap;
	grid_map->loadMap( "./test.bmp" );
	
	grid_map->showMap();

	// ---------------- 2, ----------------- //
	float sigma = 0.2f;
	float rand = 0.01f;
	std::cout<<"caculate the likelihood now ...."<<std::endl;
        MeasurementModel *measurement_model = new MeasurementModel( grid_map, sigma, rand );
        std::cout<<"likelihood map caculated finished ..."<<std::endl;
	measurement_model->displayLikelihoodMap();

	return 0;
}
