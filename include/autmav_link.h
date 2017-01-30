#include <string.h>
#include <stdint.h>
#include <iostream>
#include <vector>
#include <boost/asio.hpp>

using namespace std;
using namespace boost;

class autmav_link{
public:
	autmav_link(asio::serial_port *input_port);
	void parser(void(*)(vector <uint8_t>*));
	bool calc_checksums(vector <uint8_t>*, uint8_t *, uint8_t *);
	void kill();
	void decode(int);

private:
	asio::serial_port *port;

	bool kill_parser;
};
