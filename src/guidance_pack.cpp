/*  
This file is part of Marvel project designed and developed by Ali Jameei 
and Mohammad hossein kazemi in AUT-MAV team from Amirkabir University of 
Technology. 
    Author : Ali Jameei
    E-Mail : celarco.group@gmail.com 
*/

#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <string.h>
#include <time.h>
#include "guidance_pack.h"
#include "pid.h"
#include <ros/ros.h>
#include "autmav_msgs/Server.h"
#include "autmav_msgs/Guidance_Command.h"
#include "autmav_msgs/Navigation_Output.h"
#include <vector>
#include "std_msgs/Bool.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <boost/thread/thread.hpp>


// Guidance variables

vertical_mode g_vertical_mode;
horizontal_mode g_horizontal_mode;
heading_mode g_heading_mode;

float v_x_setpoint = 0.0, v_y_setpoint = 0.0, v_z_setpoint = 0.0;
float v_x = 0.0, v_y = 0.0, v_z = 0.0;

float heading_setpoint = 0.0;
float heading = 0.0;

float roll_setpoint = 0.0;
float roll = 0.0;

float pitch_setpoint = 0.0;
float pitch = 0.0;

float x_setpoint = 0.0, y_setpoint = 0.0, z_setpoint = 0.0;
float x = 0.0, y = 0.0, z = 0.0;

float yaw_rate_setpoint = 0.0;
float yaw_rate = 0.0;

bool armed = false;
bool server_ready = false;
bool navigation_ready = false;
bool stand_by = true;
bool shut_down = false;
bool full = false;
bool semi = false;
bool first_block_started = false;
bool desired_block_started = false;
bool cancel = false;
bool continue_blocks = false;
bool block_canceled = false;
bool new_block_recieved = false;

pid pid_x, pid_y, pid_z, pid_v_x, pid_v_y, pid_v_z;
pid pid_heading, pid_yaw_rate;

stringstream convert2string;

float PID_X_P = 1, PID_X_I = 0, PID_X_D = 0,
    PID_Y_P = 1, PID_Y_I = 0, PID_Y_D = 0,
    PID_Z_P = 1, PID_Z_I = 0, PID_Z_D = 0,
    PID_V_X_P = 0.2, PID_V_X_I = 0, PID_V_X_D = 0,
    PID_V_Y_P = 0.2, PID_V_Y_I = 0, PID_V_Y_D = 0,
    PID_V_Z_P = 0.2, PID_V_Z_I = 0, PID_V_Z_D = 0,
    PID_H_P = 1, PID_H_I = 0, PID_H_D = 0,
    PID_YAW_RATE_P = 1, PID_YAW_RATE_I = 0, PID_YAW_RATE_D = 0;

float nominal_hover_throttle = 50.0;

ros::Publisher STRING_pub;

// Flight plan variables

vector <block_struct> blocks;

unsigned short int current_block_no = 0;

Vangaurd::Navigation_Output nav_out;

void Read_Flight_Plan();
void start_navigation();
void start_server();
void set_gains();
void send_string(string);

// Server receive callback function

void server_receive_Callback(const Vangaurd::Server::ConstPtr& msg) {
	armed = msg->armed;
	server_ready = msg->ready;
}

void nav_receive_Callback(const Vangaurd::Navigation_Output::ConstPtr& msg) {
	nav_out = *msg;
	/*cout << "\tx: " << msg->x << " y: " << msg->y << " z: " << msg->z << " phi: " << msg->phi << " theta: " << msg->theta << " psi: " << msg->psi 
		   << "\tvx: " << msg->vx << " vy: " << msg->vy << " vz: " << msg->vz << " phi_dot: " << msg->phi_dot << " theta_dot: " << msg->theta_dot 
		   << " psi_dot: " << msg->psi_dot << endl << endl;*/
	navigation_ready = true;
}

void gains_receive_Callback(const std_msgs::Int16MultiArray::ConstPtr& msg){
	PID_X_P = msg->data.at(0);
    PID_X_I = msg->data.at(1);
    PID_X_D = msg->data.at(2);
    PID_Y_P = msg->data.at(3);
    PID_Y_I = msg->data.at(4);
    PID_Y_D = msg->data.at(5);
    PID_Z_P = msg->data.at(6);
    PID_Z_I = msg->data.at(7);
    PID_Z_D = msg->data.at(8);
    PID_V_X_P = msg->data.at(9);
    PID_V_X_I = msg->data.at(10);
    PID_V_X_D = msg->data.at(11);
    PID_V_Y_P = msg->data.at(12);
    PID_V_Y_I = msg->data.at(13);
    PID_V_Y_D = msg->data.at(14);
    PID_V_Z_P = msg->data.at(15);
    PID_V_Z_I = msg->data.at(16);
    PID_V_Z_D = msg->data.at(17);
    PID_H_P = msg->data.at(18);
    PID_H_I = msg->data.at(19);
    PID_H_D = msg->data.at(20);
    PID_YAW_RATE_P = msg->data.at(21);
    PID_YAW_RATE_I = msg->data.at(22);
    PID_YAW_RATE_D = msg->data.at(23);

	set_gains();
}

void command_full_receive_Callback(const std_msgs::String::ConstPtr& msg){
	full = true;
	semi = false;

	if(msg->data == "start"){
		stand_by = false;
	}
	else if(msg->data == "cancel"){
		cancel = true;
		continue_blocks = false;
		block_canceled = false;
	}
	else if(msg->data == "continue"){
		if(cancel){
			cancel =  false;
			continue_blocks = true;
		}
	}
	else if(msg->data == "stand_by"){
		stand_by = true;
	}
	else if(msg->data == "shut_down"){
		shut_down = true;
	}
}

void command_semi_receive_Callback(const std_msgs::String::ConstPtr& msg){
	full = false;
	semi = true;

	if(msg->data == "cancel"){
		cancel = true;
		continue_blocks = false;
		block_canceled = false;
	}
	else if(msg->data == "continue"){
		if(cancel){
			cancel =  false;
			continue_blocks = true;
		}
	}
	else if(msg->data == "shut_down"){
		shut_down = true;
	}
}

void block_number_full_receive_Callback(const std_msgs::Int16::ConstPtr& msg){
	full = true;
	semi = false;

	current_block_no = msg->data;
	new_block_recieved = true;	
}

void block_number_semi_receive_Callback(const std_msgs::Int16::ConstPtr& msg){
	full = false;
	semi = true;

	current_block_no = msg->data;
	desired_block_started = true;
}

// Main program start

int main(int argc, char **argv) {
   
    ros::init(argc, argv, "guidance_pack");
    ros::NodeHandle n;
    ros::Subscriber SERVER_sub = n.subscribe("server", 10, server_receive_Callback);
    ros::Subscriber NAV_sub = n.subscribe("/AUTMAV/Navigation", 10, nav_receive_Callback);
	ros::Subscriber PID_sub = n.subscribe("/AUTMAV/pid_gains", 10, gains_receive_Callback);
	ros::Subscriber COMMAND_FULL_sub = n.subscribe("/AUTMAV/guidance_command_full", 10, command_full_receive_Callback);
	ros::Subscriber COMMAND_SEMI_sub = n.subscribe("/AUTMAV/guidance_command_semi", 10, command_semi_receive_Callback);
	ros::Subscriber BN_FULL_sub = n.subscribe("/AUTMAV/block_number_full", 10, block_number_full_receive_Callback);
	ros::Subscriber BN_SEMI_sub = n.subscribe("/AUTMAV/block_number_semi", 10, block_number_semi_receive_Callback);
    ros::Publisher GUIDANCE_pub = n.advertise<Vangaurd::Guidance_Command>("guidance_pack", 10);
	STRING_pub = n.advertise<std_msgs::String>("/AUTMAV/string_msg", 10);
    Vangaurd::Guidance_Command guidance_command;

		//setting initial pid gains

	set_gains();
    
		// Wait for the server to be ready
	
	std::cout<<"Waiting for server to be ready ..."<<std::endl;
	send_string(string("Waiting for server to be ready ..."));    

	boost::thread server_th(start_server);
	
	while(!server_ready) {
		ros::spinOnce();
	}
	
	std::cout<<"Server ready!"<<std::endl;
	send_string(string("Server ready!"));	

		// Wait for navigation to be ready
	
	std::cout<<"Waiting for navigation to be ready ..."<<std::endl;
	send_string(string("Waiting for navigation to be ready ..."));
    
	boost::thread nav_th(start_navigation);
	
	while(!navigation_ready) {
		ros::spinOnce();
	}
	
	std::cout<<"navigation ready!"<<std::endl;
	send_string(string("navigation ready!"));
	
		//Reading parameters from flight_plan	
	
	std::cout<<"reading flight_plan ..."<<std::endl;
	send_string(string("reading flight_plan ..."));

	Read_Flight_Plan();

	std::cout<<"reading flight_plan done! we have " << blocks.size() << " blocks." <<std::endl;
	convert2string << blocks.size();
	send_string(string("reading flight_plan done! we have " + convert2string.str() + " blocks."));	
	convert2string.str("");

    	// Autopilot initialization (ARM)
	
	guidance_command.arm = 1;
	guidance_command.mode = 0;
	heading_setpoint = heading; //whats this????????
	guidance_command.roll = 0;
    guidance_command.pitch = 0;
	guidance_command.yaw = 0;
	guidance_command.throttle = 0;

	//GUIDANCE_pub.publish(guidance_command);

	ros::Rate loop_rate(60);
    	
    	// Guidance loop

    std::cout<<"running..."<<std::endl;
	send_string(string("running..."));

    clock_t start, end;

	block_struct temp_block;
	block_struct general_position_hold;
		general_position_hold.type = BLOCK_POSITION_HOLD;
		general_position_hold.h_mode = HORIZONTAL_VELOCITY;
		general_position_hold.v_x_setpoint = 0.0;
		general_position_hold.v_y_setpoint = 0.0;
		general_position_hold.v_mode = VERTICAL_POS;
		general_position_hold.hdg_mode = HEADING_ANGLE;
		general_position_hold.stop_con = TIME_LIMIT;
		general_position_hold.time_condition = 1000000;
		

    while(true) {
		if(shut_down){
			break;
		}
		if(stand_by){
			guidance_command.roll = 0;
    		guidance_command.pitch = 0;
			guidance_command.yaw = 0;
			guidance_command.throttle = 0;
		}
		else{
			if(full){
				if(current_block_no == 0 && !first_block_started){
					cout << "*** first block of full autonomouse flight is in the loop! ***" << endl;
					send_string(string("*** first block of full autonomouse flight is in the loop! ***"));
					if(blocks.size() == 0){
						stand_by = true;
						continue;
					}
					temp_block = blocks.at(current_block_no);
					start = clock();
					first_block_started = true;
				}
				if(cancel && !block_canceled ){
					cout << "*** block number " << current_block_no << " canceled! ***" << endl;
					convert2string << current_block_no;
					send_string(string("*** block number " + convert2string.str() + " canceled! ***"));
					convert2string.str("");

					temp_block = general_position_hold;	
					start = clock();
					block_canceled = true;
				}
				if(continue_blocks){
					cout << "*** block number " << current_block_no << " continued... ***" << endl;
					convert2string << current_block_no;
					send_string(string("*** block number " + convert2string.str() + " continued... ***"));
					convert2string.str("");

					if(current_block_no >= blocks.size()){
						stand_by = true;
						continue;
					}
					
					temp_block = blocks.at(current_block_no);
					start = clock();
					continue_blocks = false;
				}
				if(new_block_recieved){
					cout << "*** block number " << current_block_no << " chosen... ***" << endl;
					convert2string << current_block_no;
					send_string(string("*** block number " + convert2string.str() + " chosen... ***"));
					convert2string.str("");

					if(current_block_no >= blocks.size()){
						stand_by = true;
						continue;
					}

					temp_block = blocks.at(current_block_no);
					start = clock();
					new_block_recieved = false;
					if(cancel)
						cancel = false;
				}
				if(blocks.at(current_block_no).done){
					cout << "*** block number " << current_block_no << " is done! ***" << endl;
					convert2string << current_block_no;
					send_string(string("*** block number " + convert2string.str() + " is done! ***"));
					convert2string.str("");

					current_block_no++;
					if(current_block_no >= blocks.size()){
						stand_by = true;
						continue;
					}
					
					temp_block = blocks.at(current_block_no);
					start = clock();
				}
			}
			if(semi){
				if(temp_block.done){
					temp_block = general_position_hold;
					start = clock();
				}
				if(cancel && !block_canceled ){
					temp_block = general_position_hold;
					start = clock();
					block_canceled = true;
				}
				if(continue_blocks){
					temp_block = blocks.at(current_block_no);
					start = clock();
					continue_blocks = false;
				}
				if(!desired_block_started){
					temp_block = blocks.at(current_block_no);
					start = clock();
					desired_block_started = true;
					if(cancel)
						cancel = false;
				}
			}

				//vertical PID

			switch(temp_block.v_mode){
	
				case VERTICAL_POS:
					guidance_command.throttle = 100 * pid_z.loop_once((temp_block.z_setpoint - nav_out.z), 0) + nominal_hover_throttle;
				break;

				case VERTICAL_VELOCITY:
					guidance_command.throttle = 100 * pid_v_z.loop_once((temp_block.v_z_setpoint - nav_out.vz), 0) + nominal_hover_throttle;
				break;

			}

				//horizontal PID

			switch(temp_block.h_mode){

				case HORIZONTAL_VELOCITY:
					guidance_command.roll = 100 * pid_v_x.loop_once((temp_block.v_x_setpoint - nav_out.vx), 0);
			
					guidance_command.pitch = 100 * pid_v_y.loop_once((temp_block.v_y_setpoint - nav_out.vy), 0);
	
				break;

				case HORIZONTAL_POS:

				break;

				case HORIZONTAL_POS_KNOWN:

				break;

				case HORIZONTAL_ATTITUDE:

				break;

				case HORIZONTAL_PRECISION:

				break;

			}

				//heading PID

			switch(temp_block.hdg_mode){

				case HEADING_RATE:
					guidance_command.yaw = 100 * pid_yaw_rate.loop_once((temp_block.yaw_rate_setpoint - nav_out.psi_dot), 0);
				break;

				case HEADING_ANGLE:

				break;

				case HEADING_ANGLE_GLOBAL:
					guidance_command.yaw = 100 * pid_heading.loop_once((temp_block.psi_setpoint - nav_out.psi), 0);
				break;

				case HEADING_FREE:

				break;

			}

				//stop condition check

			switch(temp_block.type){

				case BLOCK_TAKE_OFF:
					if(abs(temp_block.z_setpoint - nav_out.z) < 0.1){
						if(full)
							blocks.at(current_block_no).done = true;
						if(semi)
							temp_block.done = true;
					}
				break;

				case BLOCK_LAND:
					if(abs(0.0 - nav_out.z) < 0.1){
						if(full)
							blocks.at(current_block_no).done = true;
						if(semi)
							temp_block.done = true;
					}
				break;

				case BLOCK_POSITION_HOLD:
					end = clock();
					if((float)(end - start)/CLOCKS_PER_SEC > temp_block.time_condition && temp_block.stop_con == TIME_LIMIT){
						if(full)
							blocks.at(current_block_no).done = true;
						if(semi)
							temp_block.done = true;
					}
				break;
			}
		}

			//publishing guidance command

		GUIDANCE_pub.publish(guidance_command);
        ros::spinOnce();
		loop_rate.sleep();
         
    }

    	//DISARM

    guidance_command.arm = 0;
	guidance_command.mode = 0;
	heading_setpoint = heading; //whats this????????
	guidance_command.roll = 0;
    guidance_command.pitch = 0;
	guidance_command.yaw = 0;
	guidance_command.throttle = 0;

	GUIDANCE_pub.publish(guidance_command);

	system("rosnode kill /optical_flow");
	system("rosnode kill /navigation");
	system("rosnode kill /server");

	cout << "Mission Accomplished!" << endl;
	send_string(string("Mission Accomplished!"));

    return 0;
}

void Read_Flight_Plan(){
	
	blocks.resize(0);

	FileStorage fs("/home/autmav/ROS_CATKIN_WS/src/Vanguard/test_flight_plan.yaml", FileStorage::READ);
	
	int c = 1;
	bool blocks_finished = true;
	stringstream convert;

	//Extracting take off blocks
	while(1){
		
		block_struct temp_block;
	
	blocks_finished = true;

		convert.str("");
		convert << c;
		string block_name_to = "take_off_" + convert.str();
		string block_name_l = "landing_" + convert.str();
		string block_name_ph = "position_hold_" + convert.str(); 

		FileNode take_off = fs[block_name_to];
		FileNode landing = fs[block_name_l];
		FileNode position_hold = fs[block_name_ph];

		if(!take_off.empty()){
			
			//cout << block_name_to << "\t";
			temp_block.type = BLOCK_TAKE_OFF;

			FileNodeIterator it = take_off.begin() , it_end = take_off.end();

			for( ; it != it_end; ++it){
				cout << (float)(*it)["height"] << "\t";
				cout << (float)(*it)["speed"] << "\t";
				cout << (string)(*it)["horizontal_mode"] << "\t";
				cout << (float)(*it)["roll"] << "\t";
				cout << (float)(*it)["pitch"] << "\t";
				cout << (string)(*it)["heading_mode"] << "\n"; 

				temp_block.v_mode = VERTICAL_VELOCITY;
				temp_block.v_z_setpoint = (float)(*it)["speed"] ;
				temp_block.z_setpoint = (float)(*it)["height"] ;

				if ((string)(*it)["horizontal_mode"] == "Hold_Pos"){
					temp_block.h_mode = HORIZONTAL_POS ;
				}
				if ((string)(*it)["horizontal_mode"] == "Zero_Velo"){
					temp_block.h_mode = HORIZONTAL_VELOCITY;
					temp_block.v_x_setpoint = 0.0;
					temp_block.v_y_setpoint = 0.0;
				}
				else if ((string)(*it)["horizontal_mode"] == "Hold_Att"){
					temp_block.h_mode = HORIZONTAL_ATTITUDE;
					temp_block.roll_setpoint = (float)(*it)["roll"];
					temp_block.pitch_setpoint = (float)(*it)["pitch"];
				}

				if ((string)(*it)["heading_mode"] == "Zero_Rate") {
					temp_block.hdg_mode = HEADING_RATE;
					temp_block.yaw_rate_setpoint = 0.0;
				}
				else if((string)(*it)["heading_mode"] == "Hold_Angle"){
					temp_block.hdg_mode = HEADING_ANGLE;
				}
				else if((string)(*it)["heading_mode"] == "Free"){
					temp_block.hdg_mode = HEADING_FREE;
				}
				
			}	

			blocks_finished = false;
		}

		if(!landing.empty()){
			
			//cout << block_name_l << "\t";
			temp_block.type = BLOCK_LAND;

			FileNodeIterator it = landing.begin() , it_end = landing.end();

			for( ; it != it_end; ++it){
				cout << (float)(*it)["speed"] << "\t";
				cout << (string)(*it)["horizontal_mode"] << "\t";
				cout << (string)(*it)["heading_mode"] << "\t";
				cout << (string)(*it)["object_dir"] << "\t";
				cout << (string)(*it)["method"] << "\t";
				cout << (float)(*it)["roll"] << "\t";
				cout << (float)(*it)["pitch"] << "\t";
				cout << (float)(*it)["known_x"] << "\t";
				cout << (float)(*it)["known_y"] << "\t";
				cout << (int)(*it)["h_min"] << "\t";
				cout << (int)(*it)["h_max"] << "\t";
				cout << (int)(*it)["s_min"] << "\t";
				cout << (int)(*it)["s_max"] << "\t";
				cout << (int)(*it)["v_min"] << "\t";
				cout << (int)(*it)["v_max"] << "\t";
				cout << (int)(*it)["camera_angle"] << "\n";

				temp_block.v_mode = VERTICAL_VELOCITY;
				temp_block.v_z_setpoint = (-1) * (float)(*it)["speed"];

				if ((string)(*it)["horizontal_mode"] == "ZeroVelo"){
					temp_block.h_mode = HORIZONTAL_VELOCITY;
					temp_block.v_x_setpoint = 0.0;
					temp_block.v_y_setpoint = 0.0;
				}
				else if ((string)(*it)["horizontal_mode"] == "CurrPos"){
					temp_block.h_mode = HORIZONTAL_POS ;
				}
				else if ((string)(*it)["horizontal_mode"] == "KnownPos"){
					temp_block.h_mode = HORIZONTAL_POS_KNOWN;
					temp_block.x_setpoint = (float)(*it)["known_x"];
					temp_block.y_setpoint = (float)(*it)["known_y"];
				}
				else if ((string)(*it)["horizontal_mode"] == "HoldAtt"){
					temp_block.h_mode = HORIZONTAL_ATTITUDE;
					temp_block.roll_setpoint = (float)(*it)["roll"];
					temp_block.pitch_setpoint = (float)(*it)["pitch"];
				}
				else if ((string)(*it)["horizontal_mode"] == "Prec"){
					temp_block.h_mode = HORIZONTAL_PRECISION;
					if ((string)(*it)["precision_method"] == "Color_Tracking"){
						temp_block.p_method = COLOR_TRACKING;
						temp_block.color_track.h_min = (int)(*it)["h_min"];
						temp_block.color_track.h_max = (int)(*it)["h_max"];
						temp_block.color_track.s_min = (int)(*it)["s_min"];
						temp_block.color_track.s_max = (int)(*it)["s_max"];
						temp_block.color_track.v_min = (int)(*it)["v_min"];
						temp_block.color_track.v_max = (int)(*it)["v_max"];

					}
					else if ((string)(*it)["precision_method"] == "H_Marker"){
						temp_block.p_method = H_MARKER;
						temp_block.h_detect.obj_dir = (string)(*it)["object_dir"];
						temp_block.h_detect.method = (string)(*it)["method"];
						temp_block.camera_angle = 0;
					}
					else if((string)(*it)["precision_method"] == "Object_Tracking"){
						temp_block.p_method = OBJECT_TRACKING;
						temp_block.obj_track.obj_dir = (string)(*it)["object_dir"];
						temp_block.obj_track.method = (string)(*it)["method"];
						temp_block.camera_angle = (int)(*it)["camera_angle"];

					}
					
				}

				if ((string)(*it)["heading_mode"] == "CurrAng"){
					temp_block.hdg_mode = HEADING_ANGLE;
				}
				else if ((string)(*it)["heading_mode"] == "Free"){
					temp_block.hdg_mode = HEADING_FREE;
				}
			}


			blocks_finished = false;
		}

		if(!position_hold.empty()){
			
			//cout << block_name_ph << "\t";

			temp_block.type = BLOCK_POSITION_HOLD;

			FileNodeIterator it = position_hold.begin() , it_end = position_hold.end();

			for( ; it != it_end; ++it){
				cout << (string)(*it)["horizontal_mode"] << "\t";
				cout << (string)(*it)["heading"] << "\t";
				cout << (string)(*it)["stop_condition"] << "\t";
				cout << (float)(*it)["global_angle"] << "\t";
				cout << (float)(*it)["rate"] << "\t";
				cout << (float)(*it)["turns"] << "\t";
				cout << (float)(*it)["time"] << "\t";
				cout << (float)(*it)["delta_angle"] << "\n"; 

				temp_block.v_mode = VERTICAL_POS;

				if ((string)(*it)["horizontal_mode"] == "CurrPos"){
					temp_block.h_mode = HORIZONTAL_POS;
				}
				else if ((string)(*it)["horizontal_mode"] == "ZeroVelo"){
					temp_block.h_mode = HORIZONTAL_VELOCITY;
					temp_block.v_x_setpoint = 0.0;
					temp_block.v_y_setpoint = 0.0;
				}				

				if ((string)(*it)["heading"] == "Ang"){
					temp_block.hdg_mode = HEADING_ANGLE_GLOBAL;
					temp_block.psi_setpoint = (float)(*it)["global_angle"];
				}
				else if((string)(*it)["heading"] == "Rate"){
					temp_block.hdg_mode = HEADING_RATE;
					temp_block.yaw_rate_setpoint = (float)(*it)["rate"];
					if ((string)(*it)["stop_condition"] == "Time"){
						temp_block.stop_con = TIME_LIMIT;
						temp_block.time_condition = (float)(*it)["time"];
					}
					else if((string)(*it)["stop_condition"] == "Turn"){
						temp_block.stop_con = TURNS_NUM;
						temp_block.angle_condition = ((float)(*it)["turns"])*360 ;
					}
				    else if ((string)(*it)["stop_condition"] == "Angle"){
				    	temp_block.stop_con = ANGLE_LIMIT;
						temp_block.angle_condition = (float)(*it)["delta_angle"];
					}
				}
				else if((string)(*it)["heading"] == "HoldAng"){
					temp_block.hdg_mode = HEADING_ANGLE;
				} 
					
			}	

			blocks_finished = false;
		}

		if(blocks_finished)
			break;

		blocks.push_back(temp_block);

		c++;

	}

}

void start_navigation(){
	system("./navigation");
}

void start_server(){
	system("./server");
}

void set_gains(){
	cout << "setting gains..." << endl;
	send_string(string("setting gains..."));

	pid_x.set_coeff(PID_X_P, PID_X_I, PID_X_D);
	pid_y.set_coeff(PID_Y_P, PID_Y_I, PID_Y_D);
	pid_z.set_coeff(PID_Z_P, PID_Z_I, PID_Z_D);
	pid_v_x.set_coeff(PID_V_X_P, PID_V_X_I, PID_V_X_D);
	pid_v_y.set_coeff(PID_V_Y_P, PID_V_Y_I, PID_V_Y_D);
	pid_v_z.set_coeff(PID_V_Z_P, PID_V_Z_I, PID_V_Z_D);
	pid_heading.set_coeff(PID_H_P, PID_H_I, PID_H_D);
	pid_yaw_rate.set_coeff(PID_YAW_RATE_P, PID_YAW_RATE_I, PID_YAW_RATE_D);
}

void send_string(string str){
	std_msgs::String string_msg;
	string_msg.data = str.data();
	STRING_pub.publish(string_msg);
}
