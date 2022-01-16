#include "localizer.h"

Localizer::Localizer (  Robot* robot, 
			map::GridMap* map, 
			MotionModel* motion_model, 
			MeasurementModel* measurement_model , 
			size_t nParticles ) :
    			robot_ ( robot ), 
			map_ ( map ), 
			motion_model_ ( motion_model ), 
			measurement_model_ ( measurement_model ),
			nParticles_ ( nParticles )
{
    	is_init_ = false;

    	/* 初始化粒子 */
    	const float minX = 0;
    	const float maxX = map->getSizeX();
    	const float minY = 0;
    	const float maxY = map->getSizeY();
 
	std::cout<<"min x = "<<minX<<std::endl;
	std::cout<<"max X = "<<maxX<<std::endl;
	std::cout<<"min Y = "<<minY<<std::endl;
        std::cout<<"max Y = "<<maxY<<std::endl;
	   
	cv::RNG rng ( cv::getTickCount() );
    
	size_t N = 0;
    	float weight = 1.0 / nParticles;
    
	while ( N < nParticles ){
	        float x = rng.uniform ( minX, maxX );
        	float y = rng.uniform ( minY, maxY );
	        float th = rng.uniform ( -M_PI, M_PI );

        	/* 判断是否在地图内 & 判断是否在可行区域内 */
	        float bel = map->getGridBelInt( x, y );
        	
		if ( bel != 0.0 ){ //0.0的区域是可行驶区域
            		continue;
        	}

	        /* 构造一个粒子 */
        	N++;
	        Particle pt ( Pose2d ( x * 0.1, y * 0.1, th ), weight );
        	particles_.push_back ( pt );
    	}
}

void Localizer::motionUpdate ( const Pose2d& odom )
{
    	const double MIN_DIST = 0.02; // TODO param 如果运动太近的话就不更新了，防止出现角度计算错误
    	if ( !is_init_ ){ //首次
	        last_odom_pose_ = odom;
        	is_init_ = true;
	        return;
    	}

    	/* 计算ut */
	float dx = odom.x_ - last_odom_pose_.x_;
    	float dy = odom.y_ - last_odom_pose_.y_;
    	float delta_trans = sqrt ( dx * dx + dy * dy );
    	float delta_rot1 = atan2 ( dy,  dx ) - last_odom_pose_.theta_;
    
	Pose2d::NormAngle ( delta_rot1 );
    	/* 处理纯旋转 */
    	if(delta_trans < 0.01) 
       		delta_rot1 = 0;
    
	float delta_rot2 = odom.theta_ - last_odom_pose_.theta_ - delta_rot1;
    	Pose2d::NormAngle ( delta_rot2 );
    
    	for ( size_t i = 0; i < nParticles_; i ++ ){
        	motion_model_->sampleMotionModelOdometry ( delta_rot1, delta_trans, delta_rot2, particles_.at ( i ).pose_ ); // for each particle
    	}

    	last_odom_pose_ = odom;
}


void Localizer::measurementUpdate ( const sensor::LaserScan& scan )
{
    	if ( !is_init_ ){
       		return;
    	}

    	/* 获取激光的信息 */
    	const float& ang_min = scan.angle_min;
    	const float& ang_max = scan.angle_max;
    	const float& ang_inc = scan.angle_increment;
    	const float& range_max = scan.range_max;
    	const float& range_min = scan.range_min;

    	for ( size_t np = 0; np < nParticles_; np ++ ){
	        Particle& pt = particles_.at ( np );
        
		/* for every laser beam */
	        for ( size_t i = 0; i < scan.size(); i ++ ){
            	
			/* 获取当前beam的距离 */
            		const float& R = scan.ranges[ i ];
            		if ( R > range_max || R < range_min )
                		continue;

	            	float angle = ang_inc * i + ang_min;
        	    	float cangle = cos ( angle );
            		float sangle = sin ( angle );
            
			Eigen::Vector2f p_l ( R * cangle, R * sangle ); //在激光雷达坐标系下的坐标

            		/* 转换到世界坐标系下 */
            		Pose2d laser_pose = pt.pose_ * robot_->T_r_l_;
            		Eigen::Vector2f p_w = laser_pose * p_l;
           
            		/* 更新weight */
            		float likelihood = measurement_model_->getGridLikelihood ( p_w ( 0 ), p_w ( 1 ) );
            
            		/* 整合多个激光beam用的加法， 用乘法收敛的太快 */
            		pt.weight_ += likelihood;
        	}// for every laser beam
    	} // for every particle

    	/* 权重归一化 */
    	normalization();
    
    	/* TODO 这里最好使用书上的Argument 的重采样 + KLD重采样方法. 
     	*重采样频率太高会导致粒子迅速退化*/
    	static int cnt = -1;
    	if  ( (cnt ++)  % 10 == 0 ) //减少重采样的次数
        	reSample();    //重采样
}

void Localizer::normalization()
{
	if ( !is_init_ ){
        	return;
    	}

    	long double sum = 0;
    	for ( size_t i = 0; i < nParticles_; i ++ ){
        	Particle& pt = particles_.at ( i );
        
        	/* 走出地图的粒子干掉 */
        	float bel = map_->getGridBel( pt.pose_.x_, pt.pose_.y_ );
       	 	if (!bel )
            		pt.weight_ = 0.0;
//         	else if (bel != 0.0) //如果走到未知区域就给一个很低的权值
//             		pt.weight_ = 1.0 / nParticles_;
        
        	sum += pt.weight_;
    	}

    	long double eta = 1.0 / sum;
    	for ( size_t i = 0; i < nParticles_; i ++ ){
        	Particle& pt = particles_.at ( i );
        	pt.weight_ *= eta;
    	}
}


void Localizer::reSample()
{
    	if ( !is_init_ )
        	return;

    	/* 重采样 */
    	std::vector<Particle> new_particles;
    	cv::RNG rng ( cv::getTickCount() );
    	long double inc = 1.0 / nParticles_;
    	long double r = rng.uniform ( 0.0, inc );
    	long double c = particles_.at ( 0 ).weight_;
    	int i = 0;
    
    	for ( size_t m = 0; m < nParticles_; m ++ ){
        	long double U = r + m * inc;
        	while ( U > c ){
            		i = i+1;
            		c = c + particles_.at ( i ).weight_;
        	}
        	particles_.at ( i ).weight_ = inc;
        
        	/* 新采样的粒子加了高斯，稍微增加一下多样性 */
        	Particle new_pt = particles_.at(i);
        	new_pt.pose_.x_ += rng.gaussian(0.02);
        	new_pt.pose_.y_ += rng.gaussian(0.02);
        	new_pt.pose_.theta_ += rng.gaussian(0.02);
        	Pose2d::NormAngle(new_pt.pose_.theta_);
        	new_particles.push_back ( new_pt );
    	}
    	particles_ = new_particles;
}

void Localizer::displayMap() const
{
	int sizeX = map_->getSizeX();
	int sizeY = map_->getSizeY();

	std::cout<<"localizer: map size x: "<<sizeX<<std::endl;
	std::cout<<"localizer: map size y: "<<sizeY<<std::endl;

	cv::Mat image = cv::Mat( sizeX, sizeY, CV_8UC3, cv::Scalar::all(125) );

	/*for( int i = 0; i < 1000; i ++ ){
                for( int j = 0; j < 1000; j ++ ){
                        std::cout<<"map["<<i<<", "<<j<<"]: "<<map_->getGridBelInt( i, j )<<std::endl;
                        cv::waitKey(10);
                }
        }*/

	int count = 0;
	for( int x = 0; x < sizeX; x ++ ){
                for( int y = 0; y < sizeY; y ++ ){
			cv::Vec3b p;
			if( map_->getGridBelInt(x, y) == 1.0 ){
				p[0] = 0;
                        	p[1] = 0;
                        	p[2] = 0;
			}	
			else if( map_->getGridBelInt(x, y) == 0.0 ){
				p[0] = 255;
                                p[1] = 255;
                                p[2] = 255;
			}
			else if( map_->getGridBelInt(x, y) == 0.5 ){
				p[0] = 125;
                                p[1] = 125;
                                p[2] = 125;
			}
			image.at<cv::Vec3b>(x, y) = p;
		}	
	}

	std::cout<<"particles number : "<<nParticles_<<std::endl;
	for( int i = 0; i < nParticles_; i ++ ){
		//cv::circle(image, cv::Point2d(particles_[i].pose_.x_ * 10, particles_[i].pose_.y_ * 10), 2, cv::Scalar(0, 0, 255), -1);
		
		cv::Vec3b p;
                p[0] = 0;
                p[1] = 0;
                p[2] = 255;
		image.at<cv::Vec3b>(particles_[i].pose_.x_ * 10, particles_[i].pose_.y_ * 10) = p;	
		//std::cout<<"pose of particles["<<i<<"] = "<<particles_[i].pose_.x_ * 10<<", "<<particles_[i].pose_.y_ * 10<<std::endl;
	}

	cv::imshow("map", image);
}	
	
void Localizer::displayMapWithScan( const Pose2d &pose, const sensor::LaserScan &scan ) const
{
	int sizeX = map_->getSizeX();
        int sizeY = map_->getSizeY();

        std::cout<<"localizer: map size x: "<<sizeX<<std::endl;
        std::cout<<"localizer: map size y: "<<sizeY<<std::endl;

        cv::Mat image = cv::Mat( sizeX, sizeY, CV_8UC3, cv::Scalar::all(125) );

	for( int x = 0; x < sizeX; x ++ ){
                for( int y = 0; y < sizeY; y ++ ){
                        cv::Vec3b p;
                        if( map_->getGridBelInt(x, y) == 1.0 ){
                                p[0] = 0;
                                p[1] = 0;
                                p[2] = 0;
                        }
                        else if( map_->getGridBelInt(x, y) == 0.0 ){
                                p[0] = 255;
                                p[1] = 255;
                                p[2] = 255;
                        }
                        else if( map_->getGridBelInt(x, y) == 0.5 ){
                                p[0] = 125;
                                p[1] = 125;
                                p[2] = 125;
                        }
                        image.at<cv::Vec3b>(x, y) = p;
                }
        }

	// dispaly scan
	const float angle_min = scan.angle_min;
	const float angle_max = scan.angle_max;
	const float angle_inc = scan.angle_increment;
	const float range_max = scan.range_max;
	const float range_min = scan.range_min;
	for ( size_t i = 0; i < scan.size(); i ++ ){
		const float r = scan.ranges[i];
		if( r > range_max || r < range_min ){
			continue;
		}
	
		float angle = angle_inc * i + angle_min;
		Eigen::Vector2f p_l( r * cos( angle ), r * sin( angle ) );	
	
		Eigen::Matrix2f R;
		R << cos(pose.theta_), -sin(pose.theta_),
		     sin(pose.theta_), cos(pose.theta_);
		
		Eigen::Vector2f T( pose.x_, pose.y_ );
		
		Eigen::Vector2f p_w = R * p_l + T;
		
		cv::circle(image, cv::Point2f( p_w[0] * 10 , p_w[1] * 10 ), 2, cv::Scalar(255, 0, 0), -1);
	}

	cv::imshow( "test", image );
}

