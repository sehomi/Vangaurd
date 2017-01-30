#include "autmav_link.h"

autmav_link::autmav_link(asio::serial_port *input_port){
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

void message_callback(vector <uint8_t> *buff){
	cout << "message recieved" << endl;
}

int main(){
	asio::io_service ios;
	asio::serial_port port(ios);
	port.open("/dev/ttyAMA0");
    port.set_option(asio::serial_port_base::baud_rate(57600));

	autmav_link al(&port);
	al.parser(&message_callback);
	
	return 0;
}
