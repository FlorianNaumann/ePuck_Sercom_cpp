/*
* -*- coding: utf-8 -*-
*
* 		ePuck.cpp
*
* 		Copyright 2014 Florian Maximilian Naumann <naumann.florian@mytum.de>
*
* 		This program is free software; you can redistribute it and/or modify
* 		it under the terms of the GNU General Public License as published by
* 		the Free Software Foundation; either version 3 of the License, or
* 		(at your option) any later version.
*
* 		This program is distributed in the hope that it will be useful,
* 		but WITHOUT ANY WARRANTY; without even the implied warranty of
* 		MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* 		GNU General Public License for more details.
*
* 		You should have received a copy of the GNU General Public License
* 		along with this program; if not, write to the Free Software
* 		Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
* 		MA 02110-1301, USA.
*/

#include "ePuck.h"

// 
// Private methods
// 

int ePuck::debug(const char *txt,const char* txt2){
	/**
	Show debug information and data, only works if debug information
	is enable (see "set_debug()")
	
	@param 	txt: Data to be showed separated by comma
	@type	txt: Any
	*/

	if (m_debug){
		if(txt2 != 0)
			std::cerr << "[ePuck]: " << txt << txt2 << std::endl;
		else
			std::cerr << "[ePuck]: " << txt << std::endl;
	}
	return 0;
}

std::string ePuck::recv(int n){
	/**
	Receive data from the robot
	
	@param	n: 	Number of bytes you want to receive
	@type	n: 	int
	@return: 	Data received from the robot as string if it was successful, throw an exception if not
	@rtype:		String
	@throws Exception:	If there is a communication problem
	**/
	std::string rline("");
	char *line = NULL;
	
	//std::cout << "recv" << std::endl;

	if ( !m_connection_status )
	{
		throw std::runtime_error("There is no connection");
	}

	try
	{
		line = new char[n];
		memset(line,0,n);
		int tmp;
		sync();
		tmp = read(m_device_id, line, n);
		//std::cout << "read gives " << tmp << " characters." << std::endl;
		if (tmp != -1)
		{
			for(int i=0; i<tmp; i++)
				rline += line[i];
			m_messages_received += 1;
		}
		else
		{
			if(errno == EAGAIN){
				//perror("recv()_read()");
				std::cout << '.' << std::flush;
				usleep(100);
			}
			else
				perror("recv()_read()");
			// dont do anything here

		}
		delete[] line;
		line = NULL;
	}
	catch(std::exception &e)
	{
		std::string txt("Serial communication problem: ");
		txt += e.what();
		debug(txt.c_str());
		if(line != NULL){
			delete[] line;
			line = NULL;
		}
		
		throw std::runtime_error(txt);
	}
	
	return rline;
}

int ePuck::send(const char* message, int bytes){
	/**
	Send data to the robot
	
	@param	message: Message to be sent
	@type	message: String
	@return: Number of bytes sent if it was successful. -1 if not
	@rtype:	int
	*/
	int n = 0;

	if ( !m_connection_status )
		throw std::runtime_error("There is not connection");

	// fix to end binary communication with \0
	if( (signed char)message[0] < 0 ) // binary indicator
	{
		m_last_msg_is_binary = true;
	}
	else // this message is ascii mode
	{
		if( m_last_msg_is_binary )
			write(m_device_id, "\0", 1);
		m_last_msg_is_binary = false;
	}

	try{
		n = write(m_device_id, message, bytes);
		m_messages_sent += 1;
	}
	catch(std::exception& e)
	{
		debug("Send problem:", e.what());
		return -1;
	}
	
	return n;
}

//def _read_image(self):
	/**
	Returns an image obtained from the robot's camera. For communication
	issues you only can get 1 image per second
	
	:return: The image in PIL format
	:rtype: PIL Image
	*/

	// Thanks to http://www.dailyenigma.org/e-puck-cam.shtml for
	// the code for get the image from the camera
/*	msg = struct.pack(">bb", -ord("I"), 0)

	try:
		n = self._send(msg)
		self._debug("Reading Image: sending " + repr(msg) + " and " + str(n) + " bytes")

		// We have to add 3 to the size, because with the image we
		// get "mode", "width" and "height"
		size = self._cam_size + 3
		img = self._recv(size)
		while len(img) != size:
			img += self._recv(size)

		// Create the PIL Image
		image = Image.frombuffer("RGB", (self._cam_width, self._cam_height),
								 img, "raw",
								 "BGR;16", 0, 1)

		image = image.rotate(180)
		self._pil_image = image

	except Exception, e:
		self._debug('Problem receiving an image: ', e)*/

//def _refresh_camera_parameters(self):
	/**
	Method for refresh the camera parameters, it's called for some
	private methods
	*/
/*	try:
		msg = self.send_and_receive("I").split(',')
	except:
		return False
	else:
		self._cam_mode, \
		self._cam_width, \
		self._cam_height, \
		self._cam_zoom, \
		self._cam_size = [int(i) for i in msg[1:6]]

		self._camera_parameters = self._cam_mode, self._cam_width, self._cam_height, self._cam_zoom
		*/

void ePuck::write_actuators(){
	/**
	Write in the robot the actuators values. Don't use directly,
	instead use 'step()'
	*/

	// Not all messages reply with ACK, only Ascii messages
	// acks = ['j', 't']

	// We get an iterator of the actuators list
	std::list<int*>::iterator list_it = m_actuators_to_write.begin();
	
	for(;list_it != m_actuators_to_write.end(); list_it++)
	{
		//std::cout << "write actuators list: (*list_it)[1]=" << char((*list_it)[1]) << std::endl;
		//std::cout << "laenge=" << (*list_it)[0] << std::endl;
		if (  *list_it == NULL ) continue;
		if ( (*list_it)[1] == 'L' ){
			// Leds
			// Transform String to byte array
			char msg[3];
			msg[0] = -('L'); // -(*list_it)[0];
			msg[1] = (*list_it)[2] & 0xff;
			msg[2] = (*list_it)[3] & 0xff;
			int n = send(msg,3);
			std::string tmp;
			char buf[5];
			sprintf(buf,"[%d]",n);
			tmp.append(buf);
			tmp.append(" bytes: ");
			for(int i=0; i<3;i++){
				sprintf(buf,"0x%02x,",msg[i]);
				tmp.append(buf);
			}

			debug("Binary message sent of ",tmp.c_str());
		}
		else if ( (*list_it)[1] == 'D' ){
			// Set motor speed
			// build a message of 5 bytes in little endian
			char msg[5];
			msg[0] = -('D');
			msg[1] = (*list_it)[2] & 0xff;
			msg[2] = (*list_it)[2] >> 8;
			msg[3] = (*list_it)[3] & 0xff;
			msg[4] = (*list_it)[3] >> 8;
			send(msg,5);
			std::string tmp;
			char buf[5];
			for(int i=0; i<5;i++){
				sprintf(buf,"0x%02x,",msg[i]);
				tmp.append(buf);
			}
			debug("Binary message sent of [5] bytes: ",tmp.c_str());
		}
		else{
			// Others actuators, parameters are separated by commas
			std::string msg;
			std::string reply;
			char buf[10];
			for (int i=1; i <= (*list_it)[0]; i++)
			{
				if(i==1) sprintf(buf,"%c",(*list_it)[i]); // first one is a character
				else	 sprintf(buf,"%d",(*list_it)[i]); // all others are numbers
				msg.append(buf);
				msg.append(1,',');
			}
			msg.resize(msg.size()-1);  // cut off the trailing comma
			
			reply = send_and_receive(msg.c_str());
			//if( reply.compare('j') == 0 ) // Camera
			//	refresh_camera_parameters();

			//if ( reply.compare('j') != 0 && reply.compare('t') != 0 ) // Camera and Sound
			//	debug("Unknown ACK reply from ePuck: ", reply)
		}
		delete[] *list_it;
		*list_it = NULL;
	}
	m_actuators_to_write.clear();
	return;
}

std::string ePuck::send_binary_mode(char cmd, unsigned int len_reply)
{
	// Auxiliary function for sent messages in binary modes
	// Parameters: ('Char to be sent', 'Size of reply waited')

	char msg[2] = {-cmd, 0};

	std::string tmp;
	char buf[5];
	sprintf(buf,"0x%02x,",msg[0]);
	tmp.append(buf);
	sprintf(buf,"0x%02x",msg[1]);
	tmp.append(buf);

	debug("Sending binary message: ", tmp.c_str());
	if(send(msg, 2) != 2)
		perror("send_binary_mode, send()");

	// get answer
	std::string reply = recv(len_reply);

	int tries = 0;
	while (reply.length() < len_reply && tries < 20){
		tries++;
		reply += recv(len_reply);
	}
	if(tries == 20)
		debug("timeout while issuing command:",&cmd);
	//reply = struct.unpack(parameters[2], reply)

	debug("Binary message received: ", reply.c_str());
	return reply;
}

void ePuck::read_sensors()
{
	/**
	This method is used for read the ePuck's sensors. Don't use directly,
	instead use 'step()'
	*/

	// We can read sensors in two ways: Binary Mode and Ascii Mode
	// Ascii mode is slower than Binary mode, therefore, we use
	// Binary mode whenever we can. Not all sensors are available in
	// Binary mode

	std::string reply;
	
	// Read different sensors
	if( m_sensors_to_read & SENSOR_ACCELEROMETER )
	{
		// Accelerometer sensor in a non filtered way
		if( m_accelerometer_filtered )
		{
			reply = send_binary_mode('A',12);
			m_accelerometer[0] = (reply[0]<<24) + (reply[1]<<16) + (reply[2]<<8) + reply[3];
			m_accelerometer[1] = (reply[4]<<24) + (reply[5]<<16) + (reply[6]<<8) + reply[7];
			m_accelerometer[2] = (reply[8]<<24) + (reply[9]<<16) + (reply[10]<<8) + reply[11];
			
		}
		else
		{
			reply = send_binary_mode('a',6);
			m_accelerometer[0] = (reply[0] << 8) + reply[1];
			m_accelerometer[1] = (reply[2] << 8) + reply[3];
			m_accelerometer[2] = (reply[4] << 8) + reply[5];
		}
	}
	if( m_sensors_to_read & SENSOR_PROXIMITY )
	{
		// Proximity sensors
		reply = send_binary_mode('N', 20);
		for(int i=0; i<10; i++)
			m_proximity[i] = (reply[2*i] << 8) + (reply[2*i+1]);
	}
	if( m_sensors_to_read & SENSOR_FLOOR )
	{	
		// Floor sensors
		// reply = send_binary_mode(parameters)
		// if type(reply) is tuple and type(reply[0]) is int:
		// 	self._floor_sensors = reply
		char data[6];
		for(int i=0; i<6; i++){
			sync();
			usleep(50);
			if (write(m_i2cdev, &i, 1) != 1) {
				perror("write before read");
			}
			sync();
			usleep(50);
			if (read(m_i2cdev, &data[i], 1) != 1) {
				perror("read");
			}
		}
		m_floor_sensors[0] = (data[0] << 8) + data[1];
		m_floor_sensors[1] = (data[2] << 8) + data[3];
		m_floor_sensors[2] = (data[4] << 8) + data[5];
	}				

	if( m_sensors_to_read & SENSOR_MOTOR_POSITION ){
		// Motor position sensor
		reply = send_binary_mode('Q',4);
		m_motor_position[0] = int16_t( (reply[1] << 8) + reply[0] );
		m_motor_position[1] = int16_t( (reply[3] << 8) + reply[2] );
	}
	if( m_sensors_to_read & SENSOR_LIGHT ){
		// Light sensors
		reply = send_binary_mode('O',20);
		for(int i=0; i<10; i++)
			m_light_sensor[i] = (reply[2*i] << 8) + (reply[2*i+1]);
	}
	if( m_sensors_to_read & SENSOR_MICROPHONE ){
		// Microphone
		reply = send_binary_mode('u',6);
		m_microphone[0] = (reply[1] << 8) + reply[0];
		m_microphone[1] = (reply[3] << 8) + reply[2];
		m_microphone[2] = (reply[5] << 8) + reply[4];
	}
	if( m_sensors_to_read & SENSOR_MOTOR_SPEED ){
		// Motor Speed
		reply = send_binary_mode('E',4);
		m_motor_speed[0] = int16_t( (reply[1] << 8) + reply[0] );
		m_motor_speed[1] = int16_t( (reply[3] << 8) + reply[2] );
	}
	// if( m_sensors_to_read & SENSOR_CAMERA ){
	//	// Do nothing for the camera, is an independent process
	//}
	if( m_sensors_to_read & SENSOR_SELECTOR ){
		reply = send_and_receive("C");
		int pos;
		sscanf(reply.c_str(),"c,%d\r\n",&pos);
		m_selector = pos;
	}
	if( m_sensors_to_read & SENSOR_BATTERY ){
		reply = send_binary_mode('b',1);
		m_battery = (reply[0] == true);
	}
}

// 
// Public methods
// 

ePuck::ePuck(const char* ttydev, bool debug) // Konstruktor
{
	/**
	Constructor process
	
	@param 	ttydev: Serial device to control robot, e.g. /dev/ttyS0 
	@type	address: String
	@param 	debug: If you want more verbose information, useful for debugging
	@type	debug: Boolean

	@return: ePuck object
	*/
	
	// This dictionary has as keys the first character of the message, that
	// is used to know the number of lines. If no key for the message, 1 line is assumed

	DIC_MSG[ 'v'  ] =  2;  // Version
	DIC_MSG[ '\n' ] = 23;  // Menu
	DIC_MSG['\x0c'] =  2;  // Welcome
	DIC_MSG[ 'k'  ] =  2;  // Calibration
	DIC_MSG[ 'R'  ] =  2;  // Reset

	// Monitoring Variables
	m_messages_sent = 0;
	m_messages_received = 0;
	m_last_msg_is_binary = true;
	m_version = VERSION;
	m_debug = debug;

	// Connection Attributes
	m_ttydev = ttydev;
	memset (&m_tty_old, 0, sizeof m_tty_old);
	m_device_id = -1;

	m_connection_status = false;

	// Camera attributes
	// m_cam_width = None;
	// m_cam_height = None;
	 m_cam_enable = false;
	// m_cam_zoom = None;
	// m_cam_mode = None;
	// m_cam_size = None;

	// Sensors and actuators lists
	m_sensors_to_read = 0;
	m_actuators_to_write.clear();
	
	// Floor sensor i2c bus & device
	m_i2cdev = -1;
	m_i2cadr = 0x60;

	// Sensors
	memset(m_accelerometer,0,3);
	m_accelerometer_filtered = false;
	m_selector = 0;
	m_battery = 1; //  1 => battery ok, 0 => battery<3.4V
	memset(m_motor_speed,0,3);  // left and right motor
	memset(m_motor_position ,0,2);  // left and right motor
	//m_camera_parameters = {0, 0, 0, 0};
	memset(m_floor_sensors,0,3);
	memset(m_proximity, 0, 10);
	memset(m_light_sensor,0,8);
	memset(m_microphone ,0,3);
	// m_pil_image = None;

	// Leds
	for(int i=0; i<10; i++)
		m_leds_status[i] = false;
}

ePuck::~ePuck()
{
	debug("Calling Destructor");

	// clear the actuators write code stack
	if( m_actuators_to_write.size() != 0 )
	{
		std::list<int*>::iterator list_it = m_actuators_to_write.begin();
		for(;list_it != m_actuators_to_write.end(); list_it++)
		{
			delete[] *list_it;
			*list_it = NULL;
		}
		m_actuators_to_write.clear();
	}

	if(m_connection_status)
		disconnect();
}

bool ePuck::connect()
{
	/**
	Connect with the physic ePuck robot
	
	@return: If the connection was succesful
	@rtype: Boolean
	@except Exception: If there are a communication problem, for example, the robot is off
	*/

	if( m_connection_status )
	{
		debug("Already connected");
		return false;
	}
	try
	{
		// try to open the serial device
		m_device_id = open(m_ttydev.c_str(), O_RDWR | O_NOCTTY | O_SYNC );

		if(m_device_id < 0)
		{
			debug("No serial device at specified adress!");
			return false;
		}
		// set connection attributes
		struct termios tty;
		memset (&tty, 0, sizeof tty);
		if (tcgetattr (m_device_id, &m_tty_old) != 0)
		{
			debug("error saving the old serial connection");
			return false;
		}
		/*
		if (tcgetattr (m_device_id, &tty) != 0)
		{
			debug("error setting up the serial connection");
			return false;
		}*/

		// disable IGNBRK for mismatched speed tests; otherwise receive break
		// as \000 chars
		tty.c_iflag = 0;					// ignore break signal
		tty.c_lflag = 0;                	// no signaling chars, no echo,
											// no canonical processing
		tty.c_oflag = 0;                	// no remapping, no delays
		tty.c_cflag = (CLOCAL | CREAD  | CS8);     // 8-bit chars
		tty.c_cc[VMIN]  = 0;            	// read doesn't block
		tty.c_cc[VTIME] = 5;            	// 0.5 seconds read timeout

		// working at a baudrate of 230400 (in and out)
		cfsetospeed (&tty,  B230400);
		cfsetispeed (&tty,  B230400);

		//tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

		//tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
			                        // enable reading
		//tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
		//tty.c_cflag |= 0;
		//tty.c_cflag &= ~CSTOPB;			// only one stopbit
		//tty.c_cflag |= CRTSCTS;		// no flowcontrol

		if (tcsetattr (m_device_id, TCSAFLUSH, &tty) != 0)
		{
			debug("error from tcsetattr");
			return false;
		}

		//fcntl(m_device_id, F_SETFL, FNDELAY);
	}
	catch( std::exception &e )
	{
		std::string txt("Connection problem: \n");
		txt.append(e.what());
		debug(txt.c_str());
		throw std::runtime_error(txt);
	}

	m_connection_status = true;
	debug("Connected");

	reset();
	return true;
}

int ePuck::disconnect()
{
	/**
	Close the connection with the robot.
	
	@return: 0 if all ok
	@rtype: int
	@raise Exception: if it was a problem closing the connection
	*/

	if( m_connection_status )
	{
		try{
			// Stop the robot
			debug("Timeout: 0.5s");
			stop();
			// reset to old parameters
			if (tcsetattr (m_device_id, TCSAFLUSH, &m_tty_old) != 0)
			{
				debug("error from tcsetattr");
				return false;
			}
			sleep(1);
			// Close the socket
			if( close(m_device_id) != 0 )
				perror("close()");
			m_connection_status = false;
			debug("disconnected");
		}
		catch(std::exception &e)
		{
			std::string tmp("Closing connection problem: \n");
			tmp.append(e.what());
			throw std::runtime_error(tmp.c_str());
			return -1;
		}
	}
	return 0;
}
			
void ePuck::set_debug(bool debug){
	/*
	Set / unset debug information
	@param debug: True or False, as you want or not Debug information
	@type debug: Boolean
	*/

	m_debug = debug;
}

std::string ePuck::send_and_receive(const char *msg){
	/**
	Send an Ascii message to the robot and return the reply. You can
	use it, but I don't recommend, use 'enable()', 'disable()'
	and 'step()' instead 
	
	@param msg: The message you want to send
	@type msg:	String
	@return: Response of the robot
	@rtype: String
	*/
	
	//size_t len = strlen(msg);
	std::string szMessage(msg);
	unsigned short	lines = 0,
					tries = 0;

	// Check the connection
	if ( !m_connection_status){
		throw std::runtime_error("There is not connection");
	}

	// Make sure the Message is a string
	// message = msg;

	// Add carriage return if not
	if( szMessage.find('\n') == std::string::npos )
		szMessage.append("\n");

	// Check the lines of the waited reply
	if( DIC_MSG.count(szMessage[0]) )
	{
		lines = DIC_MSG[szMessage[0]];
	}
	else
	{
		lines = 1;
	}

	char buf[3] = "";
	sprintf(buf,"%d",lines);
	debug("Waited lines: ",buf);

	// We make 5 tries before desist
	do{
		// Send the message
		char bytes[3] = "";
		sprintf(bytes,"%d",send( szMessage.c_str(), szMessage.length() ));
		debug("Message sent:", szMessage.c_str());
		debug("Bytes sent:", bytes);

		try{
			// Receive the reply. As we want to receive a line, we have to insist
			std::string reply("");
			while(std::count(reply.begin(),reply.end(),'\n') < lines){
				reply += recv();

				if(szMessage[0] == 'R'){
					// For some reason that I don't understand, if you send a reset
					// command 'R', sometimes you recive 1 or 2 lines of 'z,Command not found\r\n'
					// Therefore I have to remove it from the expected message: The Hello message
					while( reply.find("z,Command not found\r\n") != std::string::npos ){
						debug("R into z-cmd!");
						reply.erase(reply.find("z,Command not found\r\n"),21);
					}
				}
			}
			debug("Message received: ", reply.c_str());

			while( reply.find("\r\n") != std::string::npos )
				reply.erase(reply.find("\r\n"),2);
			return reply;
		}
		catch( std::exception &e ){
			tries++;
			debug("Communication timeout, retrying");
		}
	} while(tries < 5);
	throw std::runtime_error("Could not receive reply from robot");
}

//	bool ePuck::save_image(name='ePuck.jpg'){
	/**
	Save image from ePuck's camera to disk
	
	:param name: Image name, ePuck.jpg as default
	:type name: String
	
	:return: Operation result
	:rtype:  Boolean
	**/

//		if (m_pil_image)
//			return m_pil_image.save(name);
//		else
//			return false;
//	}

std::vector<int> ePuck::get_accelerometer(){
	/**
	Return Accelerometer values in (x, y, z)
	
	@return: Accelerometer values
	@rtype: int[3]
	*/
	return std::vector<int>(m_accelerometer, m_accelerometer + sizeof(m_accelerometer) / sizeof(int) );
}

bool ePuck::is_battery_full()
{	/**
	Return the battery status (0 => <3.4V, 1 => battery ok)
	@return: battery status
	@rtype: bool
	*/
	return m_battery;
}

int ePuck::get_selector(){
	/**
	Return the selector position (0-15)
	
	@return: Selector value
	@rtype: int
	*/
	return m_selector;
}

std::vector<int> ePuck::get_motor_speed(){
	/**
	Return the motor speed. Correct values are in the range [-1000, 1000]
	
	:return: Motor speed
	:rtype: Tuple
	*/
	return std::vector<int>(m_motor_speed, m_motor_speed + sizeof(m_motor_speed) / sizeof(int) );
}

//	def get_camera_parameters(self):
	/**
	Return the camera parameters as a tuple
	(mode, width, height, zoom)
	
	:return: Camera parameters
	:rtype: Tuple
	*/
//		return self._camera_parameters

std::vector<int> ePuck::get_floor_sensors(){
	/**
	Return the floor sensors values as (left, center, right)
	
	@return: Floor sensors values
	@rtype: Tuple
	*/
	return std::vector<int>(m_floor_sensors, m_floor_sensors + sizeof(m_floor_sensors) / sizeof(int) );
}

std::vector<int> ePuck::get_proximity(){
	/**
	Return the values of the 8 proximity sensors
	
	:return: Proximity sensors values
	:rtype: Tuple
	*/
	return std::vector<int>(m_proximity, m_proximity + sizeof(m_proximity) / sizeof(int) );
}

std::vector<int> ePuck::get_light_sensor(){
	/**
	Return the value of the light sensor
	
	:return: Ligth sensor value
	:rtype: Tuple
	*/
	return std::vector<int>(m_light_sensor, m_light_sensor + sizeof(m_light_sensor) / sizeof(int) );
}

std::vector<int> ePuck::get_motor_position(){
	/**
	Return the position of the left and right motor as a tuple
	
	:return: Motor position
	:rtype: Tuple
	*/
	return std::vector<int>(m_motor_position, m_motor_position + sizeof(m_motor_position) / sizeof(int) );
}
	
std::vector<int> ePuck::get_microphone(){
	/**
	Return the volume of the three microphones
	
	:return: Microphones values
	:rtype: Tuple
	*/
	return std::vector<int>(m_microphone, m_microphone + sizeof(m_microphone) / sizeof(int) );
}

bool ePuck::is_connected(){
	/**
	Return a boolean value that indicate if the robot is connected to the PC
	
	@return: If the robot is connected to the PC
	@rtype: Boolean
	*/
	return m_connection_status;
}

//	def get_image(self):
	/**
	Return the last image captured from the ePuck's camera (after a 'step()'). 
	None if	there are not images captured. The image is an PIL object
	
	:return: Image from robot's camera
	:rtype: PIL
	*/
//		return self._pil_image

std::string ePuck::get_sercom_version(){
	/**
	@return: Writes the ePuck's firmware version
	@rtype:  String
	*/
	return send_and_receive("v");
	
}

void ePuck::set_accelerometer_filtered(bool filter){
	/**
	Set filtered way for accelerometer, False is default value
	at the robot start
	
	@param filter: True or False, as you want
	@type filter: Boolean
	*/
	m_accelerometer_filtered = filter;
}

int ePuck::disable(int sensors){
	/**
	Sensor(s) that you want to get disable in the ePuck
	
	:param sensors: SENSOR-constants specified in the header. Multiple sensors can be connected with binary OR
	:type sensors: int
	:return: Sensors enabled
	:rtype: int
	:except Exception: Some wrong happened
	*/
	
	try{
		if( (sensors & 0x1ff) != sensors ){
			debug("unspecified sensors cannot be disabled!");
			sensors &= 0x1ff;	// shorten to known sensor-bits
		}

		//if(sensor & "SENOSR_CAMERA")
		//	m_cam_enable = false;

		m_sensors_to_read &= ~sensors;
	}
	catch(std::exception &e){
		debug("Something wrong happened to disable the sensors: ", e.what());
	}

	return get_sensors_enabled();
}

int ePuck::enable(int sensors){
	/**
	Sensor(s) that you want to get enable in the ePuck
	
	@param sensors: Name of the sensors, take a look to the constants defined in the header. Multiple sensors can be connected via binary OR
	@type sensors: int
	@return: Sensors enabled
	@rtype: int
	@except Exception: Some wrong happened
	*/

	// Using the * as a parameters, we get a tuple with all sensors
	try{
		if( (sensors & 0x1ff) != sensors ){
			debug("unspecified sensors cannot be enabled!");
			sensors &= 0x1ff;	// shorten to known sensor-bits
		}

		/*if sensor == "camera":
			// If the sensor is the Camera, then we refresh the
			// camera parameters
			if not self._cam_enable:
				try:
					self._refresh_camera_parameters()
					self._cam_enable = True
					self.timestamp = time.time()
				except:
					break
					*/
		if( (sensors & SENSOR_FLOOR) && (m_i2cdev == -1) ){
			// Initialize i2c bus
			m_i2cdev = open("/dev/i2c-3",O_RDWR | O_SYNC);
			if (ioctl(m_i2cdev, I2C_SLAVE, m_i2cadr) < 0)
			{
				// error handling
				debug("There is no i2c floor sensor on bus{3} adr{0x60} to use!");
				sensors &= ~SENSOR_FLOOR; // do not enable
			}
			
		}
		m_sensors_to_read |= sensors;
		debug("Sensors enabled");
	}
	catch(std::exception &e){
		debug("Something wrong happened to enable the sensors: ", e.what());
	}
	return get_sensors_enabled();
}

int ePuck::get_sensors_enabled(){
	/**
	:return: Returns a bit-code of sensors that are active; use the SENSOR_XXX constants and binary AND to check which are enabled
	:rtype: int
	*/
	return m_sensors_to_read;
}

void ePuck::set_motors_speed(int l_motor,int r_motor){
	/**
	Set the motors speed. The MAX and MIN speed of the ePcuk is [-1000, 1000]
	
	@param l_motor: Speed of left motor
	@type l_motor: int
	@param r_motor: Speed of right motor
	@type r_motor: int
	*/

	// I don't check the MAX and MIN speed because this check
	// will be made by the ePuck's firmware. Here we need speed
	// and we lose time mading recurrent chekings

	int *tmp = new int[4];
	tmp[0] = 3;
	tmp[1] = 'D';
	tmp[2] = l_motor;
	tmp[3] = r_motor;
	m_actuators_to_write.push_back(tmp);
}

void ePuck::set_motor_position(int l_wheel, int r_wheel){
	/**
	Set the motor position, useful for odometry
	
	@param l_wheel: left wheel
	@type l_wheel: int
	@param r_wheel: right wheel
	@type r_wheel: int
	*/

	int *tmp = new int[4];
	tmp[0] = 3;
	tmp[1] = 'P';
	tmp[2] = l_wheel;
	tmp[3] = r_wheel;
	m_actuators_to_write.push_back(tmp);
	//m_actuators_to_write.append(("P", l_wheel, r_wheel))
}

bool ePuck::set_led(unsigned int led,unsigned int value){
	/**
	Turn on/off the leds
	
	:param led_number: If led_number is other than 0-7, all leds are set to the indicated value.
	:type led_number: int
	:param led_value: 
		- 0 : Off
		- 1 : On (Red)
		- 2 : Inverse, does not work for all leds <- !!!
	:type led_value: int
	*/

	if (led <= 7)
	{
		int *tmp = new int[4];
		tmp[0] = 3;
		tmp[1] = 'L';
		tmp[2] = led;
		tmp[3] = value;
		m_actuators_to_write.push_back(tmp);

		if(value == 0)
			m_leds_status[led] = false;
		else if(value == 1)
			m_leds_status[led] = true;
		else
			m_leds_status[led] = !m_leds_status[led];
	}
	else
	{
		int *tmp = new int[4];
		tmp[0] = 3;
		tmp[1] = 'L';
		tmp[2] = 10;
		tmp[3] = value;
		m_actuators_to_write.push_back(tmp);

		for(int i=0; i<8; i++){
			if(value == 0)
				m_leds_status[i] = false;
			else if(value == 1)
				m_leds_status[i] = true;
		}

	}

	return true;
}

//bool ePuck::set_body_led(unsigned int led_value)
//{
	/**
	Turn on /off the body led
	
	@param led_value: 
		- 0 : Off
		- 1 : On (green)
		- 2 : Inverse
	@type led_value: int
	*/
/*
	int *tmp = new int[4];
	tmp[0] = 3;
	tmp[1] = 'L';
	tmp[2] = 8;
	tmp[3] = led_value;
	m_actuators_to_write.push_back(tmp);

	if (led_value == 0)
		m_leds_status[8] = false;
	else if(led_value == 1)
		m_leds_status[8] = true;
	else
		m_leds_status[8] = !m_leds_status[8];
		
	return true;
}*/


//bool ePuck::set_front_led(unsigned int led_value)
//{
	/**
	Turn on /off the front led
	
	:type	led_value: int
	:param 	led_value: 
		- 0 : Off
		- 1 : On (green)
		- 2 : Inverse
	*/
/*
	int *tmp = new int[4];
	tmp[0] = 3;
	tmp[1] = 'L';
	tmp[2] = 9;
	tmp[3] = led_value;
	m_actuators_to_write.push_back(tmp);

	if(led_value == 0)
		m_leds_status[9] = false;
	else if(led_value == 1)
		m_leds_status[9] = true;
	else
		m_leds_status[9] = !m_leds_status[9];
		
	return true;
}
*/
//void ePuck::set_sound(int sound){
	/**
	Reproduce a sound
	
	@param sound: Sound in the range [1,5]. Other for stop
	@type sound: int
	*/
/*
	int *tmp = new int[3];
	tmp[0]= 2;
	tmp[1]='T';
	tmp[2]=sound;
	m_actuators_to_write.push_back(tmp);
}*/


//	def set_camera_parameters(self, mode, width, height, zoom):
	/**
	Set the camera parameters
	
	:param mode: GREY_SCALE, LINEAR_CAM, RGB_365, YUM
	:type  mode: String
	:param width: Width of the camera
	:type  width: int
	:param height: Height of the camera
	:type  height: int
	:param zoom: 1, 4, 8
	:type  zoom: int
	*/
/*
	if mode in CAM_MODE:
		self._cam_mode = CAM_MODE[mode]
	else:
		self._debug(ERR_CAM_PARAMETERS, "Camera mode")
		return -1

	if int(zoom) in CAM_ZOOM:
		self._cam_zoom = zoom
	else:
		self._debug(ERR_CAM_PARAMETERS, "Camera zoom")
		return -1

	if self.conexion_status and int(width) * int(height) <= 1600:
		// 1600 are for the resolution no greater than 40x40, I have
		// detect some problems
		self._actuators_to_write.append(("J",
										 self._cam_mode,
										 width,
										 height,
										 self._cam_zoom))
		return 0
*/

bool ePuck::calibrate_proximity_sensors(){
	/**
	Calibrate proximity sensors, keep off any object in 10 cm
	
	@return: Successful operation
	@rtype: Boolean
	*/

	std::string reply = send_and_receive("k");
	if (reply[0] == 'k')
		return true;
	else
		return false;
}

bool ePuck::reset(){
	/**
	Reset the robot
	
	@return: Successful operation
	@rtype: Boolean
	@raise Exception: If there is not connection
	*/
	if ( !m_connection_status )
	{
		throw std::runtime_error("There is no connection");
	}
	debug(send_and_receive("R").c_str());

	return true;
}

bool ePuck::stop(){
	/**
	Stop the motor and turn off all leds
	@return: Successful operation
	@rtype: Boolean
	@raise Exception: If there is not connection
	*/

	std::string reply;
	
	if ( !m_connection_status )
		throw std::runtime_error("There is no connection");

	reply = send_and_receive("S");
	debug(reply.c_str());

	if( reply.compare("s") == 0 )
		return true;
	else
		return false;
}

void ePuck::step(){
	/**
	Method to update the sensor readings and to reflect changes in 
	the actuators. Before invoking this method is not guaranteed
	the consistency of the sensors
	*/

	if( !m_connection_status )
		throw std::runtime_error("There is no connection");

	write_actuators();
	read_sensors();

	// Get an image in 1 FPS
	// if( m_cam_enable && (time() - m_timestamp) > 1 )
	// {
	// 	read_image();
	// 	m_timestamp = time();
	// }
}
