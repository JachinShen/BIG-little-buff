#include "ros/ros.h"
#include "std_msgs/Int16.h"

#include <sstream>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

#include <fcntl.h>
#include <iostream>

#include <OpenUart.h>

#define SMALL_RUNE 1
#define BIG_RUNE 2
#define STOP_RUNE 3

int mode = 3;

void praseData(const char * data, int size){
    if (size < 4)
		return;
    int start = 0;
    while(1){
		if (start >= size - 3)
	    	return;
		if (data[start] == 0xFF && data[start+3] == 0xFE){
	    	break;
		}
		++start;
    }
    ++start;

    unsigned char cmd1 = (unsigned char) data[start];
    unsigned char cmd2 = (unsigned char) data[start+1];

    switch(cmd1){
		case 1:{
			mode = SMALL_RUNE;
	    	std::cout << "small rune" << std::endl;
	   		break;
		}
		case 2:{
			mode = BIG_RUNE;
	    	std::cout << "big rune" << std::endl;
	    	break;
		}
		case 3:{
			mode = STOP_RUNE;
			std::cout << "stop" << std::endl;
			break;
		}
		default:{
			mode = STOP_RUNE;
			std::cout << "stop" << std::endl;
			break;
		}
    }

    ++start;
    if (start + 1 < size){
		praseData(data+start+1, size-start-1);
    }	
}

void readCommand(){
    char buf[255]={0};
    size_t bytes = 0;
    ioctl(fd, FIONREAD, &bytes);
    if (bytes>0 && bytes < 255)
        bytes = read(fd, buf, bytes);
    else if (bytes >= 255)
		bytes = read(fd, buf, 255);
    else
		return;
   
    praseData(buf, bytes);
}

void paraReceiver(){
    while(1){
		readCommand();
        usleep(1000);
    }
}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "readuart");


  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Int16>("new", 1);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    std_msgs::Int16 msg;

    paraReceiver();
    
	msg.data = mode;
	
	std::cout << msg.data<<std::endl;

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
