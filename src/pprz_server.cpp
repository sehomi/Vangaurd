#include <iostream>
#include <vector>
#include <stdint.h>
#include "ros/ros.h"
#include "autmav_link.h"
#include "autmav_msgs/Server.h"

using namespace std;

autmav_link al;
ros::Publisher server_pub;

attitude at;
lidar li;
imu_gyro_scaled imu;
imu_accel_scaled imu_accel;

void message_callback(vector <uint8_t> *buff){

	switch(buff->at(3)){
	
		case ATTITUDE_MSG_ID:
			al.attitude_msg_decode(buff, &at);
			//cout << at.phi * 180/3.1415 << endl;
		break;

		case LIDAR_MSG_ID:
			al.lidar_msg_decode(buff, &li);
			//cout << li.distance << endl;
		break;

		case IMU_GYRO_SCALED_ID:
			al.imu_gyro_scaled_msg_decode(buff, &imu);
			//cout << imu.p << endl;
		break;

		case IMU_ACCEL_SCALED_ID:
			al.imu_accel_scaled_msg_decode(buff, &imu_accel);
			//cout << imu_accel.ax << endl;
		break;
	}

	Vangaurd::Server server_msg;
	server_msg.phi = at.phi * 180/3.1415;
	server_msg.psi = at.psi * 180/3.1415;
	server_msg.theta = at.theta * 180/3.1415;
	server_msg.p = imu.p;
	server_msg.q = imu.q;
	server_msg.r = imu.r;
	server_msg.x_acc = imu_accel.ax;
	server_msg.y_acc = imu_accel.ay;
	server_msg.z_acc = imu_accel.az;
	server_msg.lidar = li.distance;
	server_pub.publish(server_msg);
}

int main(int argc, char **argv){
	asio::io_service ios;
	asio::serial_port port(ios);
	port.open("/dev/ttyAMA0");
    port.set_option(asio::serial_port_base::baud_rate(57600));

	ros::init(argc, argv, "pprz_server");
    ros::NodeHandle n;
    server_pub = n.advertise<Vangaurd::Server>("/AUTMAV/server", 10);

	al.init(&port);
	al.parser(&message_callback);
	
	return 0;
}
