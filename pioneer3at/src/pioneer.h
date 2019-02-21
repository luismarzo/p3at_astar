//
// C++ Interface: Pioneer
//
// Description:
//
//
// <luismarzo@gmail.com>
//
//

#ifndef PIONEER
#define PIONEER

// #include <SerialStream.h>
// #include <gazebo/physics/physics.hh>
// #include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class Pioneer
{
  public:

	enum direction { FORWARD, BACKWARD, LEFT, RIGHT };

	Pioneer(int argc, char **argv)
	{	ros::init(argc, argv, "path planning");
		ros::NodeHandle n;
		pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
		ros::Rate loop_rate();
		msg.linear.x = 0;
		msg.angular.z = 0; //inicialización
		pub.publish(msg);
		ros::spinOnce();
	};

	void go_forward()
	{	std::cout<<"forward"<<std::endl;
		msg.linear.x = 0.425;
		msg.angular.z = 0;
		upgrade();
	}

	void go_backward()
	{	std::cout<<"backward"<<std::endl;
		msg.linear.x = -0.425;
		msg.angular.z = 0;
		upgrade();
	}

	void go_diag()
	{	std::cout<<"diag"<<std::endl;
		msg.linear.x = 0.586;
		msg.angular.z = 0;
		upgrade();
	}

	void turn_for45()
	{	std::cout<<"turn45"<<std::endl;
		msg.linear.x = 0;
		msg.angular.z = 0.3808;
		upgrade();
	}

	void turn_back45()
	{std::cout<<"back45"<<std::endl;
		msg.linear.x = 0;
		//msg.angular.z = -0.498;
		msg.angular.z = -0.3808;
		upgrade();
	}

	void stop()
	{	std::cout<<"stop"<<std::endl;
		msg.linear.x = 0;
		msg.angular.z = 0;
		upgrade();
	}

	void upgrade()
	{	//std::cout<<"publishing: linear:"<<msg.linear.x<<" and angular:"<<msg.angular.z<<std::endl;
		pub.publish(msg);
		ros::spinOnce();
		sleep(3);
	}

  private:
	ros::Publisher pub;
	geometry_msgs::Twist msg;
};

#endif // PIONEER