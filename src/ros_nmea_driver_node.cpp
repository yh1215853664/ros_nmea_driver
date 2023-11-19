#include <ros/ros.h>
#include <serial/serial.h>
#include <nmea_msgs/Sentence.h>

#include <string>

#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

#include <bits/stdc++.h>
#include <boost/algorithm/string.hpp>
#include <boost/assign/list_of.hpp>

#include <boost/asio.hpp>
using boost::asio::ip::tcp;

ros::Publisher gnss_pos_pub;
ros::Publisher gnss_yaw_pub;
ros::Publisher nmea_sentence_pub;

typedef enum
{
	GGA,
	TRA,
	RMC,
	UNKNOWN
} nmea_sentence_type;

//check the xor sum of a nmea sentence
bool checksum(std::string &nmea)
{
	unsigned int len = nmea.length();

	if (len < 6)
		return false;

	// $GPGGA, ... \r\n
	int pos_asterisk = nmea.find("*");
	if (pos_asterisk == std::string::npos)
		return false;

	char nmea_sum[2];
	nmea_sum[0] = nmea[pos_asterisk + 1];
	nmea_sum[1] = nmea[pos_asterisk + 2];

	unsigned int sum = strtol(nmea_sum, nullptr, 16); //string to long
	unsigned int sum_ = 0;							  //calculated checksum

	for (int i = 1; i < len - 5; i++)
	{
		sum_ ^= nmea[i];
	}

	// printf("%02x %02x\n",sum, sum_);

	return sum == sum_ ? true : false;
}

void gnss_cb(std::string &nmea)
{
	bool is_nmea = checksum(nmea);

	if (!is_nmea)
	{
		ROS_ERROR_STREAM("NMEA Sentence Check Failed!");
		ROS_INFO("Sentence : %s", nmea.c_str());
		return;
	}

	std::vector<std::string> nmea_split;
	boost::split(nmea_split, nmea, boost::is_any_of(","));

	nmea_sentence_type nst = UNKNOWN;
	if (nmea_split[0].find("GGA") != std::string::npos)
		nst = GGA;
	if (nmea_split[0].find("TRA") != std::string::npos)
		nst = TRA;
	if (nmea_split[0].find("RMC") != std::string::npos)
		nst = RMC;

	sensor_msgs::NavSatFix gnss_pos_msg;
	// nav_msgs::Odometry gnss_yaw_msg;
	sensor_msgs::NavSatStatus gnss_pos_status;
	sensor_msgs::Imu gnss_yaw_msg;

	std::string gnss_frame_id = "gnss_link";
	std::string imu_frame_id = "imu_link";

	switch (nst)
	{
	case GGA:
	{
		// ROS_INFO("GGA: %s", nmea.c_str());
		// ROS_INFO("latitude : %lf",  strtod(nmea_split[2].c_str(), nullptr));
		// ROS_INFO("longitude : %lf", strtod(nmea_split[4].c_str(), nullptr));
		// ROS_INFO("altitude : %lf",  strtod(nmea_split[9].c_str(), nullptr));

		// ROS_INFO("status : %d", strtol(nmea_split[6].c_str(), nullptr, 10));

		double degree, minute;

		degree = (int)strtod(nmea_split[2].c_str(), nullptr) / 100;
		minute = strtod(nmea_split[2].c_str(), nullptr) - degree * 100;

		gnss_pos_msg.latitude = degree + minute / 60.0;

		degree = (int)strtod(nmea_split[4].c_str(), nullptr) / 100;
		minute = strtod(nmea_split[4].c_str(), nullptr) - degree * 100;

		gnss_pos_msg.longitude = degree + minute / 60.0;

		gnss_pos_msg.altitude = strtod(nmea_split[9].c_str(), nullptr);

		int satnu = strtol(nmea_split[7].c_str(), nullptr, 10);
		gnss_pos_msg.position_covariance_type = satnu;

		double hdop = strtod(nmea_split[8].c_str(), nullptr);
		double pos_cov = hdop * hdop / 2.0;
		gnss_pos_msg.position_covariance = boost::assign::list_of(pos_cov)(0)(0)(0)(pos_cov)(0)(0)(0)(pos_cov);

		int pos_status = strtol(nmea_split[6].c_str(), nullptr, 10);
		if (pos_status == 0)
			gnss_pos_status.status = -1;
		else if (pos_status == 1)
			gnss_pos_status.status = 0;
		else if (pos_status == 2 || pos_status == 5)
			gnss_pos_status.status = 1;
		else if (pos_status == 4)
			gnss_pos_status.status = 2;

		gnss_pos_msg.status = gnss_pos_status;

		gnss_pos_msg.header.stamp = ros::Time::now();
		gnss_pos_msg.header.frame_id = gnss_frame_id;

		gnss_pos_pub.publish(gnss_pos_msg);
	}
	break;

	case TRA:
	{
		int status = strtol(nmea_split[5].c_str(), nullptr, 10);

		if (status == 4) //定向固定解 发送报文
		{
			double yaw = strtod(nmea_split[2].c_str(), nullptr);
			ROS_INFO("YAW = %lf", yaw);
			//GPS角度转换成ROS角度
			//东 0° 逆时针旋转
			yaw = 90 - yaw;
			if (yaw < 0)
			{
				yaw = yaw + 360;
			}
			yaw = yaw * M_PI / 180.0;

			geometry_msgs::Quaternion gnss_yaw_quat;
			gnss_yaw_quat = tf::createQuaternionMsgFromYaw(yaw);

			gnss_yaw_msg.orientation = gnss_yaw_quat;
			gnss_yaw_msg.header.stamp = ros::Time::now();
			gnss_yaw_msg.header.frame_id = imu_frame_id; // Important!!! Assuming this is an imu topic.

			gnss_yaw_pub.publish(gnss_yaw_msg);
		}
	}
	break;

	case RMC:

		break;

	case UNKNOWN:

		break;

	default:
		break;
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "gnss_driver");
	ros::NodeHandle nh;

	nmea_msgs::Sentence nmea_sentence;
	nmea_sentence_pub = nh.advertise<nmea_msgs::Sentence>("nmea_sentence", 1000);

	gnss_pos_pub = nh.advertise<sensor_msgs::NavSatFix>("gnss_pos", 10); // gnss 定位消息发布
	gnss_yaw_pub = nh.advertise<sensor_msgs::Imu>("gnss_yaw", 10);		 // gnss 航向消息发送

	//gnss 输出方式
	std::string connect_mode;
	ros::param::get("/ros_nmea_driver_node/connect_mode", connect_mode); //获取 connect_mode
	ROS_INFO("connect_mode = %s", connect_mode.c_str());

	if (connect_mode == "serial")
	{
		//gnss 设备串口
		serial::Serial gnss_serial;
		std::string port_name;									  //串口名称
		int baudrate;											  //串口波特率
		ros::param::get("/ros_nmea_driver_node/port", port_name); //获取串口名称
		ros::param::get("/ros_nmea_driver_node/baud", baudrate);  //获取串口波特率

		//打开串口
		try
		{
			serial::Timeout _timeout = serial::Timeout::simpleTimeout(1000);
			gnss_serial.setPort(port_name);
			gnss_serial.setBaudrate(baudrate);
			gnss_serial.setTimeout(_timeout);

			gnss_serial.open();
		}
		catch (serial::IOException &e)
		{
			ROS_ERROR_STREAM("unable to open port.");
			return -1;
		}

		while (ros::ok())
		{
			std::string ret = gnss_serial.readline();
			if (ret == "")
				continue;
			// ROS_INFO("result: %s", ret.c_str());
			nmea_sentence.sentence = ret;
			nmea_sentence_pub.publish(nmea_sentence);

			gnss_cb(ret);
		}

		//关闭串口
		gnss_serial.close();
	}
	else if (connect_mode == "tcp_client")
	{
		try
		{
			//gnss 网络端口
			boost::asio::io_service io_service;
			tcp::resolver resolver(io_service);
			std::string tc_ip;										   //tcp client ip
			int tc_port;											   // tcp client port
			ros::param::get("/ros_nmea_driver_node/tc_ip", tc_ip);	   //获取 ip
			ros::param::get("/ros_nmea_driver_node/tc_port", tc_port); //获取 端口

			ROS_INFO("Tcp Client ip: %s ,port: %d", tc_ip.c_str(), tc_port);

			auto endpoint_iterator = resolver.resolve({tc_ip.c_str(), std::to_string(tc_port).c_str()});
			tcp::socket socket(io_service);
			boost::asio::connect(socket, endpoint_iterator);

			std::string nmea_recv;
			while (ros::ok())
			{
				boost::array<char, 256> buf;
				boost::system::error_code error;

				size_t len = socket.read_some(boost::asio::buffer(buf), error);

				if (error == boost::asio::error::eof)
				{
					ROS_INFO("Connection closed cleanly by peer.");
					break; // Connection closed cleanly by peer.
				}
				else if (error)
				{
					throw boost::system::system_error(error); // Some other error.
				}

				std::string t(buf.data(), len);
				nmea_recv += t;

				// ROS_INFO("nmea_recv: %s", nmea_recv.c_str());
				std::size_t found = nmea_recv.find("\r\n");
				while (found != std::string::npos)
				{
					std::size_t nsp = nmea_recv.find("$"); // nmeasentence start pos
					std::size_t nsp_ = nmea_recv.find("#"); // nmeasentence start pos

					if(nsp_ < nsp)
						nsp = nsp_;

					// std::cout << "nsp:" << nsp << " " << "found:" << found << std::endl;

					// if(nsp > found)
					// {
					// 	std::cout << nmea_recv << std::endl;
					// }

					if (nsp != std::string::npos && nsp < found) //找到报文头 并且 报文头和尾相对位置正确
					{
						std::string nmea = nmea_recv.substr(nsp, found + 1);
						nmea_recv.erase(0, found + 1);

						nmea_sentence.sentence = nmea;
						nmea_sentence_pub.publish(nmea_sentence);

						// ROS_INFO("nmea: %s", nmea.c_str());
						if (nmea.find("#") == std::string::npos)
						{
							gnss_cb(nmea);
						}

						found = nmea_recv.find("\r\n");
					}
					else 
					{
						break;
					}
				}
			}
		}
		catch (const std::exception &e)
		{
			std::cerr << e.what() << '\n';
		}
	}
	else
	{
		ROS_INFO("Unknown Connect Mode");
	}

	return 0;
}
