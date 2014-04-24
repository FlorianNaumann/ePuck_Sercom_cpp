/*
*!/usr/bin/env python
* -*- coding: utf-8 -*-
*
* 		ePuck.py
*
* 		Copyright 2010 Manuel Martín Ortiz <manuel.martin@itrblabs.eu>
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
*
* 		-- ePuck.py --
*
* 		The aim of this library is to provide access to the ePuck robots
* 		through a bluetooth connection. Thus, you can write a program that
* 		read from the ePuck's sensors and write in their actuators, This
* 		will allow us to create advanced programs that can develop a wide
* 		variety of complex tasks. It is necesary that the ePuck has installed
* 		the Webot's fimware 1.4.2 or 1.4.3. You can find this fantastic
* 		simulator on this site: http://www.cyberbotics.com/
*
* 		This library is written in Python 2.6, and you can import it from
* 		any program written in Python  (same version or later). In addition
* 		to this, you will also need two extra libraries:
*
* 			-> Python Bluetooth or Pybluez
* 			-> Python Image Library (PIL)
*
* 		In this package you will find some examples of how to use this library.
*
* 		You may experience some problems when you work with your ePuck, We
* 		recommend you take into consideration the following special
* 		characteristic: we use a bluetooth communciation, therefore our bandwith
* 		is limited and we cannot expect to do too many tasks in short
* 		time; i.e:  If you put the wheels speed to max and want
* 		to make a quick process of the images, you will know what I'm saying.
* 		So remember, you are processing in your computer, not on the ePuck,
* 		and you need to take the sensors data and write on the actuators
* 		values on the ePuck
*
* 		For further information and updates visit http://www.itrblabs.eu
*/


//import i2c // Ground sensors are directly connected to the overo's i2c bus
#include <linux/i2c-dev.h> /* for I2C_SLAVE */
#include <linux/i2c.h>
#include <sys/ioctl.h>

#include <sys/types.h> // uint8_t
#include <inttypes.h> // uint8_t

#include <map> // replacement for dictionaries
#include <list>
#include <vector>
#include <string>
#include <cstring>
#include <algorithm>
#include <exception>
#include <stdexcept>
#include <cstdio>
#include <iostream>
// #include <ctime> // only needed for the 1 fps camera
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>

//__package__ = "ePuck" // fuer "from epuck import epuck"
//__docformat__ = "restructuredtext"

/*

TODO change to doxygen and rewrite
"""
:newfield company: Company
"""
*/
#define VERSION "1.2.2+gumstix-sercom-1.1"
/**
__author__ = "Manuel Martin Ortiz"
__license__ = "GPL"
__company__ = "RTDI - ITRB Labs"
__contact__ = ["manuel.martin@itrblabs.eu"]
*/


// You have to use the keys of this dictionary for indicate on "enable" function
// the sensor that you want to read
// std::map<std::string, char> DIC_SENSORS;
// DIC_SENSORS["accelerometer"]  = 'a';
// DIC_SENSORS["selector"]       = 'c';
// DIC_SENSORS["motor_speed"]    = 'e';
// DIC_SENSORS["camera"]         = 'i';
// DIC_SENSORS["floor"]          = 'm';
// DIC_SENSORS["proximity"]      = 'n';
// DIC_SENSORS["light"]          = 'o';
// DIC_SENSORS["motor_position"] = 'q';
// DIC_SENSORS["microphone"]     = 'u';

// define constants for enable and disable sensors
#define	SENSOR_ACCELEROMETER    0x001
#define	SENSOR_SELECTOR         0x002
#define	SENSOR_MOTOR_SPEED      0x004
#define	SENSOR_FLOOR            0x008
#define	SENSOR_PROXIMITY        0x010
#define	SENSOR_LIGHT            0x020
#define	SENSOR_MOTOR_POSITION   0x040
#define	SENSOR_MICROPHONE       0x080
#define	SENSOR_BATTERY          0x100



class ePuck
{
	/**
	This class represent an ePuck object
	*/

	private:
		std::map<char,uint8_t> DIC_MSG;

		unsigned int m_messages_sent;
		unsigned int m_messages_received;
		bool m_last_msg_is_binary;
		std::string m_version;
		bool m_debug;

		// Connection Attributes
		std::string m_ttydev;
		struct termios m_tty_old;
		int m_device_id;
		bool m_connection_status;

		// private 
		// Camera attributes
		//m_cam_width = None
		//m_cam_height = None
		bool m_cam_enable;
		//m_cam_zoom = None
		//m_cam_mode = None
		//m_cam_size = None
		
		//time_t m_timestamp;

		// Sensors and actuators lists
		int m_sensors_to_read;
		std::list<int*> m_actuators_to_write;
		
		// Floor sensor i2c handler & device adr
		int m_i2cdev; // 3
		int m_i2cadr; // 0x60

		// Sensors
		int m_accelerometer[3];
		bool m_accelerometer_filtered;
		unsigned char m_selector;
		bool m_battery;
		int m_motor_speed[2];  // left and right motor
		int m_motor_position[2];  // left and right motor
		//m_camera_parameters = (0, 0, 0, 0)
		int m_floor_sensors[3];
		int m_proximity[10];
		int m_light_sensor[10];
		int m_microphone[3];
		//m_pil_image = None

		// Leds
		bool m_leds_status[10];

		//TODO serport, vermutlich Serielles Protokoll-Objekt außer anderer Package
		
	// 
	// Private methods
	// 
	
		int debug(const char *txt,const char* txt2=0);
		std::string recv(int n=512);
		int send(const char* message, int bytes);
		std::string send_binary_mode(char cmd,unsigned int len_reply);
		// def _read_image(self);
		//def _refresh_camera_parameters(self);
		void write_actuators();		
		void read_sensors();

	// 
	// Public methods
	// 
	public:

		ePuck(const char* ttydev="/dev/ttyS0", bool debug=false);
		~ePuck();
		bool connect();
		int disconnect();
		void set_debug(bool debug);
		std::string send_and_receive(const char *msg);
	//	bool save_image(name='ePuck.jpg');
		std::vector<int> get_accelerometer(); // triple int als rueckgabe
		int get_selector();
		bool is_battery_full();
		std::vector<int> get_motor_speed();	// TODO Kapselung!! 2 ints als rueckgabe
	//	def get_camera_parameters(self) // returns 4 ints
		std::vector<int> get_floor_sensors(); // 3 * 16bit-Zahlen als rueckgabe
		std::vector<int> get_proximity(); // 10 * 8bit-Zahlen als Rueckgabe
		std::vector<int> get_light_sensor(); // returns 10 * 8bit
		std::vector<int> get_motor_position(); // 3* ints
		std::vector<int> get_microphone(); // 3* ints
		bool is_connected();
	//	def get_image(self):   //		return self._pil_image
		std::string get_sercom_version();
		void set_accelerometer_filtered(bool filter=false);
		int disable(int sensors);
		int enable(int sensors);
		int  get_sensors_enabled(); // returns a list of enabled sensors
		void set_motors_speed(int l_motor,int r_motor);
		void set_motor_position(int l_wheel,int r_wheel);
		bool set_led(unsigned int led,unsigned int value);
	//	bool set_body_led(unsigned int led_value);
	//	bool set_front_led(unsigned int led_value);
	//	void set_sound(int sound);
	//	void set_camera_parameters(char *mode, int width, int height, int zoom);		
		bool calibrate_proximity_sensors();
		bool reset();
		bool stop();	
		void step();
};
