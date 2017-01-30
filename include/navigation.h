#ifndef NAVIGATION_HEADER
#define NAVIGATION_HEADER

#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <time.h>
#include <boost/thread/thread.hpp>
#include "autmav_msgs/Navigation_Output.h"
#include "autmav_msgs/Server.h"

using namespace std;

struct of_output{
	float vx = 0.0;
	float vy = 0.0;
};

class Navigation{
	public:
		Navigation();

	private:
		void Watch_Dog();
		void start_OF();
		void of_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
		void imu_callback(const Vangaurd::Server::ConstPtr& msg);
		void imu_of_fusion();
		void of_pix_per_sec_2_m_per_sec();
		
		ros::NodeHandle *n;
		ros::Subscriber OF_sub;
		ros::Subscriber IMU_sub;
		ros::Publisher NAV_pub;
		
		bool OF_status = false;
		float OF_dt = 0.0;
		clock_t start, end;

		Vangaurd::Navigation_Output nav_out; 
		Vangaurd::Server imu;
		of_output of_out_pix_per_sec;
		of_output of_out_m_per_sec;

		float RX = 0.0, RY = 0.0, RZ = 0.0, H = 0.0, F = 0.5;
		
};

#endif
