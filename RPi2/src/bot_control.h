//#include "tserial.h"
#include <boost/asio/serial_port.hpp> 
#include <boost/asio.hpp> 
#include "blocking_reader.h"
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include "stdafx.h"  
#include <iostream>

using namespace boost;

class serial {

private:
	// private attributes
	//Tserial *com;
	asio::io_service io;
	asio::serial_port port;
public:

	
	serial():port(io) {
		

	}

	bool startDevice(char *prt)
	{
		
		port.open(prt);
		printf("Connected");
		port.set_option(asio::serial_port_base::baud_rate(9600));
		port.set_option(boost::asio::serial_port_base::character_size(8));
		port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
		port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
		port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
		std::this_thread::sleep_for(std::chrono::seconds(5));
		return 1;
		
	}

	void stopDevice()
	{
		port.cancel();
		port.close();
	

		io.stop();
		io.reset();
		// ------------------
	
	}

	void send_data(unsigned char data)
	{
		//unsigned char data = 0;

		asio::write(port, boost::asio::buffer(&data, sizeof(char)));
		printf("%c", data);

	}

	char get_data(void) {
		blocking_reader reader(port, 500);
		char c;
		reader.read_char(c);
		return c;


	}
};