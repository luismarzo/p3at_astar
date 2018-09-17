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
	bool direction0;
	bool direction90;
	bool direction180;
	bool direction270;

	Pioneer(int argc, char **argv)
	{
		ros::init(argc, argv, "a_star_publisher");
		ros::NodeHandle n;
		pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
		ros::Rate loop_rate();
		msg.linear.x = 0;
		msg.angular.z = 0; //inicialización
		pub.publish(msg);
		ros::spinOnce();
	};

	void go_forward()
	{
		msg.linear.x = 0.505;
		msg.angular.z = 0;
	}

	void go_backward()
	{
		msg.linear.x = -0.505;
		msg.angular.z = 0;
	}

	void go_diag()
	{
		msg.linear.x = 0.636;
		msg.angular.z = 0;
	}

	void turn_for45()
	{
		msg.linear.x = 0;
		msg.angular.z = 0.498;
	}


	void turn_back45()
	{
		msg.linear.x = 0;
		msg.angular.z = -0.498;
	}

	void stop()
	{
		msg.linear.x = 0;
		msg.angular.z = 0;
	}

	void upgrade()
	{
		pub.publish(msg);
		ros::spinOnce();
		
	}

  private:
	ros::Publisher pub;
	geometry_msgs::Twist msg;
};

#endif // PIONEER