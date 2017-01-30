#include "navigation.h"

int main(int argc, char **argv) {  

    ros::init(argc, argv, "navigation");
    Navigation nav;

    return 0;
}

Navigation::Navigation(){
	n = new ros::NodeHandle;

	boost::thread th(boost::bind(&Navigation::start_OF,this)); 

	cout << "OF is running ..." << endl;

	NAV_pub = n->advertise<Vangaurd::Navigation_Output>("/AUTMAV/Navigation", 10);
	OF_sub = n->subscribe("/AUTMAV/Optical_Flow", 10, &Navigation::of_callback, this);
	IMU_sub = n->subscribe("/server", 10, &Navigation::imu_callback, this);	

	Watch_Dog();

	cout << "Watch Dog is running ..." << endl;
}

void Navigation::Watch_Dog(){
	
	ros::Rate loop_rate(60);

	while(1){
			
		if(OF_dt > 1){
			cout << "restarting OF!" << endl;
			boost::thread th(boost::bind(&Navigation::start_OF,this));
		}
		
		ros::spinOnce();
		loop_rate.sleep();

	}
}

void Navigation::of_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	if(OF_status){
		start = end;
	}
	end = clock();
	if(OF_status){
		OF_dt = (float)(end - start)/CLOCKS_PER_SEC;
	}

	OF_status = true;

	of_out_pix_per_sec.vx = msg->data.at(0);
	of_out_pix_per_sec.vy = msg->data.at(1);

	of_pix_per_sec_2_m_per_sec();

	imu_of_fusion();

	nav_out.vx = msg->data.at(0);
	nav_out.vy = msg->data.at(1);

	nav_out.phi_dot = imu.phi_dot;
	nav_out.theta_dot = imu.theta_dot;
	nav_out.psi_dot = imu.psi_dot;
	nav_out.psi = imu.psi;
	nav_out.theta = imu.theta;
	nav_out.phi = imu.phi;
	nav_out.vz = imu.v_z;

	NAV_pub.publish(nav_out);
}

void Navigation::imu_callback(const Vangaurd::Server::ConstPtr& msg){
	imu = *msg;
}

void Navigation::start_OF(){
	system("cd /home/autmav/ROS_CATKIN_WS/src/Optical_Flow/build/devel/lib/Optical_Flow; ./OF");
}

void Navigation::imu_of_fusion(){

	nav_out.vx = -1 * of_out_m_per_sec.vy + RZ * imu.r; 
	nav_out.vy = of_out_m_per_sec.vx + RZ * imu.p; 

}

void Navigation::of_pix_per_sec_2_m_per_sec(){

	of_out_m_per_sec.vx = of_out_pix_per_sec.vx * H/F;
	of_out_m_per_sec.vy = of_out_pix_per_sec.vy * H/F;

}
