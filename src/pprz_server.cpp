#include <iostream>
#include <vector>
#include <stdint.h>
#include "autmav_link.h"

using namespace std;

void message_callback(vector <uint8_t> *buff){
	cout << "message recieved" << endl;
}

int main(){

	autmav_link al("/dev/ttyAMA0", 57600);
	al.parser(&message_callback);
	
	return 0;
}
