#include <ros/ros.h>
#include <ros/console.h>


#include <boost/asio/serial_port.hpp>
#include <boost/asio/basic_serial_port.hpp>
#include <boost/asio/read.hpp>
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/optional.hpp>


#include <xv11_driver/LaserMeasurements.h>
#include <sensor_msgs/LaserScan.h>

std::string frame_id, port_name;

/*
 class XV11Serial {
 boost::asio::io_service io_serv;
 boost::asio::serial_port port;

 public:
 XV11Serial() :
 io_serv(io_serv), port(io_serv, "/dev/ttyUSB0") {

 boost::asio::serial_port_base::baud_rate baud(115200);
 boost::asio::serial_port_base::character_size character_size(8);
 boost::asio::serial_port_base::flow_control flow_control(
 boost::asio::serial_port_base::flow_control::none);
 boost::asio::serial_port_base::parity parity(
 boost::asio::serial_port_base::parity::none);
 boost::asio::serial_port_base::stop_bits stop_bits(
 boost::asio::serial_port_base::stop_bits::one);
 port.set_option(baud);
 port.set_option(character_size);
 port.set_option(flow_control);
 port.set_option(parity);
 port.set_option(stop_bits);
 }

 };
 */

double to_radians(double degrees){
	return degrees * (boost::math::constants::pi<double>() / 180.0);
}

class XV11Driver {
private:

	typedef struct {
		uint16_t distance;
		uint16_t strength;
		bool invalid_data;
		bool strength_warning;
	} subdata_t;

	boost::asio::io_service io_serv;
	boost::asio::serial_port port;

	ros::NodeHandle nh;

public:
	XV11Driver(ros::NodeHandle& nh, std::string port_name) :
			nh(nh), port(io_serv, port_name) {

		boost::asio::serial_port_base::baud_rate baud(115200);
		boost::asio::serial_port_base::character_size character_size(8);
		boost::asio::serial_port_base::flow_control flow_control(
				boost::asio::serial_port_base::flow_control::none);
		boost::asio::serial_port_base::parity parity(
				boost::asio::serial_port_base::parity::none);
		boost::asio::serial_port_base::stop_bits stop_bits(
				boost::asio::serial_port_base::stop_bits::one);
		port.set_option(baud);
		port.set_option(character_size);
		port.set_option(flow_control);
		port.set_option(parity);
		port.set_option(stop_bits);

		ROS_INFO("Opened serial port: %s", port_name.c_str());
	}

	xv11_driver::LaserMeasurements read_packet(void) {
		std::vector<char> data;

		xv11_driver::LaserMeasurements msg;

		while (true) {
			char byte;
			boost::asio::read(port, boost::asio::buffer(&byte, 1));
			data.push_back(byte);
			ROS_DEBUG("Received byte: %x", byte);
			if(data[0] != (char)0xfa){
				ROS_INFO("Threw away data buffer! Framing error? Byte was: %x", data[0]);
				data.clear();
				continue;
			}
			ROS_INFO("data vector size: %d", (int)data.size());
			if(data.size() == 22){
				uint8_t start = data[0];
				uint8_t index = data[1];
				uint16_t speed = (data[3] << 8) | (data[2]);
				subdata_t subdata[4];
				for(int i=0; i<3; i++){
					subdata[i].distance = (data[4 + (4*i) + 1] << 8) | data[4 + (4*i) + 0];
					subdata[i].strength = (data[4 + (4*i) + 3] << 8) | data[4 + (4*i) + 2];
					subdata[i].invalid_data = ((subdata[i].distance & 0x8000) != 0);
					subdata[i].strength_warning = ((subdata[i].distance & 0x4000) != 0);
					subdata[i].distance &= ~(0x8000 | 0x4000);
					msg.ranges.push_back((float)(subdata[i].distance) / 1000.0); 	// the distances are given in millimeters,
																					// which we convert to meters
					msg.intensities.push_back((float)(subdata[i].strength));
				}

				uint16_t checksum = (data[21] << 8) | data[20];

				// if checksum is bad, go cry

				index -= 0xA0;
				index = (index + 45-2) % 90;

				msg.header.stamp = ros::Time::now();
				msg.packet_index = index;
				msg.angle_min = to_radians(index * 4);
				msg.angle_max = to_radians(index * 4 + 3);
				msg.angle_increment = to_radians(4);
				msg.time_increment = 1e-6;
				msg.range_min = 0.06;
				msg.range_max = 5.0;

//				break;
				return msg;
			}

		}//while

//		return msg;
	}

};

class LaserScanGenerator {

public:
	boost::optional<sensor_msgs::LaserScan> msg;

	LaserScanGenerator() {
	}

	void init_msg(void){
		sensor_msgs::LaserScan ls;
		msg = ls;
		msg->header.frame_id = frame_id;
		msg->header.stamp = ros::Time::now();
		msg->angle_min = 0;
		msg->angle_max = to_radians(359);
		msg->time_increment = 0;
		msg->scan_time = 0;
		msg->range_min = 0.06;
		msg->range_max = 5.0;
		msg->ranges.clear();
		msg->intensities.clear();
	}

	boost::optional<sensor_msgs::LaserScan> generate_packet(xv11_driver::LaserMeasurements measurement){
		if(measurement.packet_index == 0){
			init_msg();
		}
		if(!msg){
			msg.reset();
			return msg;
		}
		msg->scan_time += measurement.time_increment * 4;

		BOOST_FOREACH(float range, measurement.ranges){
			msg->ranges.push_back(range);
		}

		BOOST_FOREACH(float strength, measurement.intensities){
			msg->intensities.push_back(strength);
		}

		if(measurement.packet_index == 89){
			msg->time_increment = (msg->scan_time)/90.0;
			return msg;
		} else {
			msg.reset();
			return msg;
		}

	}

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "xv11_driver");

	ros::NodeHandle nh("~");
	ROS_INFO("Started xv11_driver_node");

	if (nh.hasParam("frame_id")) {
		nh.getParam("frame_id", frame_id);
	} else {
		ROS_ERROR("Must include _frame_id parameter!");
		return -1;
	}

	if (nh.hasParam("port")) {
		nh.getParam("port", port_name);
	} else {
		ROS_ERROR("Must include _port parameter!");
		return -1;
	}

	XV11Driver laser_object(nh, port_name);
	LaserScanGenerator laser_scan_generator;

	ros::Publisher lmp = nh.advertise<xv11_driver::LaserMeasurements>(
			"laser_measurements", 1000);
	ros::Publisher lsp = nh.advertise<sensor_msgs::LaserScan>("laser_scan",
			1000);

	xv11_driver::LaserMeasurements lmp_msg; // = xv11_driver::LaserMeasurements();
	boost::optional<sensor_msgs::LaserScan>lsp_msg; // = sensor_msgs::LaserScan();

	while (ros::ok()) {
		lmp_msg = laser_object.read_packet();
		ROS_INFO("Read LaserMeasurements from lidar");
		lsp_msg = laser_scan_generator.generate_packet(lmp_msg);
		lmp.publish(lmp_msg);
		ROS_INFO("Published LaserMeasurement");
		if(lsp_msg){
			lsp.publish(lsp_msg.get());
		}
	}

	ros::spin();
}
