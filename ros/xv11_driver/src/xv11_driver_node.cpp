#include <vector>

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

inline double to_radians(double degrees) {
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

	xv11_driver::LaserMeasurements msg; // move this here to avoid allocating new

	std::vector<char> packet_buffer;
public:
	XV11Driver(std::string port_name) :
			port(io_serv, port_name) {

		boost::asio::serial_port_base::baud_rate baud(115200);
		boost::asio::serial_port_base::character_size character_size(8);
		boost::asio::serial_port_base::flow_control flow_control(boost::asio::serial_port_base::flow_control::none);
		boost::asio::serial_port_base::parity parity(boost::asio::serial_port_base::parity::none);
		boost::asio::serial_port_base::stop_bits stop_bits(boost::asio::serial_port_base::stop_bits::one);
		port.set_option(baud);
		port.set_option(character_size);
		port.set_option(flow_control);
		port.set_option(parity);
		port.set_option(stop_bits);

		ROS_INFO("Opened serial port: %s", port_name.c_str());

		msg.ranges.reserve(4); //preallocate sizes for speed
		msg.intensities.reserve(4);
		packet_buffer.reserve(22);
	}

	xv11_driver::LaserMeasurements read_packet(void) {
		msg.ranges.clear();
		msg.intensities.clear();
		packet_buffer.clear();

		while (true) {
			char byte;
			boost::asio::read(port, boost::asio::buffer(&byte, 1));
			packet_buffer.push_back(byte);
			if (packet_buffer[0] != (char)0xfa) {
				ROS_INFO("Threw away data buffer! Framing error? Byte was: %2x", packet_buffer[0]);
				packet_buffer.clear();
				continue;
			}
			if (packet_buffer.size() == 22) {
				uint8_t start = packet_buffer[0];
				uint8_t index = packet_buffer[1];
				uint16_t speed = (packet_buffer[3] << 8) | (packet_buffer[2]);
				subdata_t subdata[4];
				for (int i = 0; i < 4; i++) {
					subdata[i].distance = (packet_buffer[4 + (4 * i) + 1] << 8) | packet_buffer[4 + (4 * i) + 0];
					subdata[i].strength = (packet_buffer[4 + (4 * i) + 3] << 8) | packet_buffer[4 + (4 * i) + 2];
					subdata[i].invalid_data = ((subdata[i].distance & 0x8000) != 0);
					subdata[i].strength_warning = ((subdata[i].distance & 0x4000) != 0);
					subdata[i].distance &= ~(0x8000 | 0x4000);
					msg.ranges.push_back((float) (subdata[i].distance) / 1000.0);
					// the distances are given in millimeters,
					// which we convert to meters
					msg.intensities.push_back((float) (subdata[i].strength));
				}

				uint16_t checksum = (packet_buffer[21] << 8) | packet_buffer[20];

				// if checksum is bad, go cry
				// if program doesn't work, go cry more
				// TODO: implement checksum checksumming?

				index -= 0xA0;
				index = (index + 45 - 2) % 90;

				msg.header.stamp = ros::Time::now();
				msg.packet_index = index;
				msg.angle_min = to_radians(index * 4);
				msg.angle_max = to_radians(index * 4 + 3);
				msg.angle_increment = to_radians(4);
				msg.time_increment = 1e-6;
				msg.range_min = 0.06;
				msg.range_max = 5.0;

				return msg;
			}

		}	//while

	}

};

class LaserScanGenerator {

public:
	boost::optional<sensor_msgs::LaserScan> msg;
	boost::optional<sensor_msgs::LaserScan> empty;
	sensor_msgs::LaserScan ls;
	LaserScanGenerator() {
		ls.ranges.reserve(360);
		ls.intensities.reserve(360);
	}

	void init_msg(void) {

		ls.header.frame_id = frame_id;
		ls.header.stamp = ros::Time::now();
		ls.angle_min = 0;
		ls.angle_max = to_radians(359);
		ls.time_increment = 0;
		ls.scan_time = 0;
		ls.range_min = 0.06;
		ls.range_max = 5.0;
		ls.ranges.clear();
		ls.intensities.clear();

		msg.reset(ls); // stick ls inside the optional

	}

	boost::optional<sensor_msgs::LaserScan> generate_packet(xv11_driver::LaserMeasurements measurement) {

		if (measurement.packet_index == 0) {
			init_msg();
		}

		if (!msg) {
			return msg;
		}

		msg->scan_time += measurement.time_increment * 4;

		BOOST_FOREACH(float range, measurement.ranges) {
			msg->ranges.push_back(range);
		}

		BOOST_FOREACH(float strength, measurement.intensities) {
			msg->intensities.push_back(strength);
		}

		if (measurement.packet_index == 89) {
			msg->time_increment = (msg->scan_time) / 90.0;
			return msg;
		} else {
			return empty; // return "blank" optional if ours isn't ready yet
		}

	}

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "xv11_driver");

	ros::NodeHandle priv_nh("~");
	ros::NodeHandle nh;
	ROS_INFO("Started xv11_driver_node");

	if (priv_nh.hasParam("frame_id")) {
		priv_nh.getParam("frame_id", frame_id);
	} else {
		ROS_ERROR("Must include _frame_id parameter!");
		return -1;
	}

	if (priv_nh.hasParam("port")) {
		priv_nh.getParam("port", port_name);
	} else {
		ROS_ERROR("Must include _port parameter!");
		return -1;
	}

	XV11Driver laser_object(port_name);
	LaserScanGenerator laser_scan_generator;

	ros::Publisher lmp = nh.advertise<xv11_driver::LaserMeasurements>("laser_measurements", 1000);
	ros::Publisher lsp = nh.advertise<sensor_msgs::LaserScan>("laser_scan", 1000);

	xv11_driver::LaserMeasurements lmp_msg;
	boost::optional<sensor_msgs::LaserScan> lsp_msg;

	while (ros::ok()) {
		lmp_msg = laser_object.read_packet();
		lsp_msg = laser_scan_generator.generate_packet(lmp_msg);
		lmp.publish(lmp_msg);
		if (lsp_msg) {
			lsp.publish(*lsp_msg);
		}
	}

	ros::spin();
}
