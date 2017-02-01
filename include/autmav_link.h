#include <string.h>
#include <stdint.h>
#include <iostream>
#include <vector>
#include <boost/asio.hpp>

using namespace std;
using namespace boost;

const float ALT_UNIT_COEF_GYRO_IMU = 0.0139882;
const float ALT_UNIT_COEF_ACCEL_IMU = 0.0009766;

#define ATTITUDE_MSG_ID 6
#define LIDAR_MSG_ID 234
#define IMU_GYRO_SCALED_ID 131  
#define IMU_ACCEL_SCALED_ID 132

struct attitude{
	float phi = 0.0;
	float theta = 0.0;
	float psi = 0.0;
};

struct lidar{
	float distance = 0.0;
}; 

struct imu_gyro_scaled{
	float p = 0.0;
	float q = 0.0;
	float r = 0.0;
};

struct imu_accel_scaled{
	float ax = 0.0;
	float ay = 0.0;
	float az = 0.0;
};  

union byte_to_float_4{
	uint8_t bytes[4];
	float result;
};

union byte_to_int_4{
	uint8_t bytes[4];
	int32_t result;
};

class autmav_link{
public:
	void init(asio::serial_port *input_port);
	void parser(void(*)(vector <uint8_t>*));

	void attitude_msg_decode(vector <uint8_t> *, attitude *);
	void lidar_msg_decode(vector <uint8_t> *, lidar *);
	void imu_gyro_scaled_msg_decode(vector <uint8_t> *, imu_gyro_scaled *);
	void imu_accel_scaled_msg_decode(vector <uint8_t> *, imu_accel_scaled *);

	void kill();
	//void attitude_msg_decode(vector <uint8_t>*, attitude*);

private:
	bool calc_checksums(vector <uint8_t>*, uint8_t *, uint8_t *);
	bool kill_parser;
	asio::serial_port *port;
};

void autmav_link::init(asio::serial_port *input_port){
	port = input_port;

	kill_parser = false;
}

void autmav_link::parser(void(*callback)(vector <uint8_t> *buff)){

	char recieved_byte;
	uint8_t check_sum_a_addr, check_sum_b_addr, length, counter; 
	uint8_t A, B;
	vector <uint8_t> temp_buff;	
	
	while(!kill_parser){
		if(asio::read(*port, asio::buffer(&recieved_byte, 1)) > 0){
			if(uint8_t(recieved_byte) == 0x99){
				temp_buff.resize(0);
				temp_buff.push_back(uint8_t(recieved_byte));
				counter = 0;
			}
			else if(counter == 1){
				check_sum_b_addr = uint8_t(recieved_byte) - 1;
				check_sum_a_addr = uint8_t(recieved_byte) - 2;
				temp_buff.push_back(uint8_t(recieved_byte));
			}
			else if(counter == check_sum_b_addr){
				temp_buff.push_back(uint8_t(recieved_byte));
				calc_checksums(&temp_buff, &A, &B);
				
				if(A == temp_buff.at(check_sum_a_addr) && B == temp_buff.at(check_sum_b_addr))
					callback(&temp_buff);
				
			}
			else{
				temp_buff.push_back(uint8_t(recieved_byte));
			}		
			counter++;
		}
	}

}

bool autmav_link::calc_checksums(vector <uint8_t> *buff, uint8_t *A, uint8_t *B){
	*A = buff->size();
	*B = buff->size();

	for(int i = 2; i < buff->size() - 2; i++){
		*A += buff->at(i);
		*B += *A;
	}
}

void autmav_link::kill(){
	kill_parser = true;
}

void autmav_link::attitude_msg_decode(vector <uint8_t>* att_buff, attitude* att){
	byte_to_float_4 btf;
	
	btf.bytes[0] = att_buff->at(4);
	btf.bytes[1] = att_buff->at(5);
	btf.bytes[2] = att_buff->at(6);
	btf.bytes[3] = att_buff->at(7);

	att->phi = btf.result;

	btf.bytes[0] = att_buff->at(8);
	btf.bytes[1] = att_buff->at(9);
	btf.bytes[2] = att_buff->at(10);
	btf.bytes[3] = att_buff->at(11);

	att->psi = btf.result;

	btf.bytes[0] = att_buff->at(12);
	btf.bytes[1] = att_buff->at(13);
	btf.bytes[2] = att_buff->at(14);
	btf.bytes[3] = att_buff->at(15);

	att->theta = btf.result;
}

void autmav_link::lidar_msg_decode(vector <uint8_t> *lidar_buff, lidar *lidar){
	byte_to_float_4 btf;

	btf.bytes[0] = lidar_buff->at(4);
	btf.bytes[1] = lidar_buff->at(5);
	btf.bytes[2] = lidar_buff->at(6);
	btf.bytes[3] = lidar_buff->at(7);

	lidar->distance = btf.result;
}

void autmav_link::imu_gyro_scaled_msg_decode(vector <uint8_t> *imu_buff, imu_gyro_scaled *imu){
	byte_to_int_4 bti;

	bti.bytes[0] = imu_buff->at(4);
	bti.bytes[1] = imu_buff->at(5);
	bti.bytes[2] = imu_buff->at(6);
	bti.bytes[3] = imu_buff->at(7);

	imu->p = bti.result * ALT_UNIT_COEF_GYRO_IMU;

	bti.bytes[0] = imu_buff->at(8);
	bti.bytes[1] = imu_buff->at(9);
	bti.bytes[2] = imu_buff->at(10);
	bti.bytes[3] = imu_buff->at(11);

	imu->q = bti.result * ALT_UNIT_COEF_GYRO_IMU;

	bti.bytes[0] = imu_buff->at(12);
	bti.bytes[1] = imu_buff->at(13);
	bti.bytes[2] = imu_buff->at(14);
	bti.bytes[3] = imu_buff->at(15);

	imu->r = bti.result * ALT_UNIT_COEF_GYRO_IMU;
}

void autmav_link::imu_accel_scaled_msg_decode(vector <uint8_t> *imu_accel_buff, imu_accel_scaled *imu_accel){
	byte_to_int_4 bti;

	bti.bytes[0] = imu_accel_buff->at(4);
	bti.bytes[1] = imu_accel_buff->at(5);
	bti.bytes[2] = imu_accel_buff->at(6);
	bti.bytes[3] = imu_accel_buff->at(7);

	imu_accel->ax = bti.result * ALT_UNIT_COEF_ACCEL_IMU;

	bti.bytes[0] = imu_accel_buff->at(8);
	bti.bytes[1] = imu_accel_buff->at(9);
	bti.bytes[2] = imu_accel_buff->at(10);
	bti.bytes[3] = imu_accel_buff->at(11);

	imu_accel->ay = bti.result * ALT_UNIT_COEF_ACCEL_IMU;

	bti.bytes[0] = imu_accel_buff->at(12);
	bti.bytes[1] = imu_accel_buff->at(13);
	bti.bytes[2] = imu_accel_buff->at(14);
	bti.bytes[3] = imu_accel_buff->at(15);

	imu_accel->az = bti.result * ALT_UNIT_COEF_ACCEL_IMU;
}
