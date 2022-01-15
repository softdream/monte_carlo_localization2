#ifndef __DATA_READ_H_
#define __DATA_READ_H_

#include "data_type.h"
#include "Pose2d.h"

namespace simulate{

class DataRead
{
public:
	DataRead()
	{

	}
	
	DataRead( const std::string &inputFile )
	{
		openSimulationFile( inputFile );
	}

	~DataRead()
	{

	}

	bool openSimulationFile( const std::string &inputFile )
	{
		input_file.open( inputFile.c_str(), std::ifstream::in );

	        if( !input_file.is_open() ){
        	        std::cout<<"Failed to open the simulation file ..."<<std::endl;
                	return false;
        	}

	        std::cout<<"............Open the Simulation File ............."<<std::endl;

	}

        void closeSimulationFile()
	{
		return input_file.close();
	}

	int readAFrameData( sensor::LaserScan &scan, Pose2d &odom )
	{	
		memset( scan.ranges, 0, scan.size() );
		std::string line;

	        std::getline(input_file, line);
        	{
                	//std::cout << line << std::endl;
	                std::istringstream iss(line);
        	        std::string tag;
                	iss >> tag;
	                std::string num;

        	        if (tag.compare("laser") == 0) {

                	        for (int i = 0; i < scan.size(); i++) {
                        	        iss >> num;
                                	//std::cout << num << "\t";
	                                //iss >> scan[count].range[i];
        	                        if (!num.compare("inf")) {
                	                        scan.ranges[i] = 65536.0f;
                        	        }
                                	else{
                                        	scan.ranges[i] = std::stof( num );
	                                }
        	                }
                	        count++;
				return 1;
	                }
			else if(tag.compare( "odom" ) == 0 ){
				iss >> num;
				odom.x_ = std::stof( num );
				iss >> num;
				odom.y_ = std::stof( num );
				iss >> num;
				odom.theta_ = std::stof( num );
			
				return 2;
			}
	        }

	}

	inline const int filePointPose()
        {
                return input_file.tellg();
        }

        inline const int endOfFile()
        {
                return input_file.eof();
        }

        inline const long getFrameCount() const
        {
                return count;
        }

private:
	std::ifstream input_file;
        long count = 0;

};

}


#endif
