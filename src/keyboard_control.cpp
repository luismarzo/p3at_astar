#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
//#include "ros/ros.h"
#include <sstream>
/* and not not_eq */
#include <iso646.h>
#include "pioneer.h"
/* add -lm to command line to compile with this header */
#include <math.h>
//#include "geometry_msgs/Twist.h"




int main(int argc, char **argv)
{
    
    
    Pioneer pioneer(argc,argv);

    Pioneer::direction direction= Pioneer::FORWARD;


    pioneer.stop();
    pioneer.upgrade();
    sleep(3);
    char key;
    while (ros::ok())
    {
        

std::cout<<"forward: w, backward: s, left: a, right:d, diagonal:e "<<std::endl;
std::cin>>key;
switch (key)
		{
		case 'w':
			pioneer.go_forward();
			break;
		case 's':
			pioneer.go_backward();
			break;
		case 'a':
			pioneer.turn_for45();
			break;
		case 'd':
			pioneer.turn_back45();
			break;
        case 'q':
			pioneer.turn_back45();
			break;
        case 'e':
			pioneer.go_diag();
			break;
		
		default:
			std::cout << "Repeat key\n"<< std::endl;
		}
        pioneer.stop();
    }

    return 0;
}
