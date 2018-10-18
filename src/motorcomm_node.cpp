#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <string>

#include <fcntl.h> // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls

// unused imports
//#include <iostream>
//#include <stdio.h> // standard input / output functions
//#include <string.h> // string function definitions
//#include <unistd.h> // UNIX standard function definitions
//#include <errno.h> // Error number definitions


#define BAUDRATE B57600

int fileDescriptor;

/* To test in ros, first run this node, then issue the command:
* rostopic pub /motor_power geometry_msgs/Twist "linear:
  x: 120.0
  y: -80.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -1

*/

geometry_msgs::Twist vel_msg;
unsigned char serialData[] = {250, 0, 0, 0, 0, 251};

void motorPowerCallback(const geometry_msgs::Twist::ConstPtr & msg)
{

	// Uncomment this if you want to be able to modify the incoming data.
	// (Also the declaration above)	
	//vel_msg = *msg; 
	//vel_msg.linear.x = 666; //e.g.

	// Example on how to access the data, both from msg and vel_msg. Note the arrow.
	// It seems msg->anglular.z is essentially the same as (*msg).angular.z
	// Note: Using the arrow the data is read only, which is normally what you need.
	// Using the pointers this way the data is never copied around, which is good for 
	// performance on the pi!
	
	// Uncomment below for debugging.
	//ROS_INFO("I got this:\n Linear:\nx:%f\ny:%f\nz:%f\n MegaAngle:\nx:%f\ny:%f\nz:%f\n",
	//	vel_msg.linear.x, vel_msg.linear.y,msg->linear.z,
	//	msg->angular.x, msg->angular.y, msg->angular.z);


	// interface used over serial is:
	// (byte values)
	//  0 meanst stop, 1 forward and 2 rev
	// SOM, L_dir, L_speed, R_dir, R_speed, EOM
	// where x_speed are between 0 and 200.
	// Issuing a stop (0 in any of the dirs) will stop both wheels.

	if(round(msg->linear.x) == 0 && round(msg->linear.y == 0)) {
		serialData[1] = 0;
		serialData[2] = 0;
		serialData[3] = 0;
		serialData[4] = 0;
	} else {
		// convert sign of power into digit 1 or 2 for arduino comm.
		serialData[1] = (unsigned char)round(1.5-0.5*abs(msg->linear.x)/(msg->linear.x));
		// remove sign from incoming data, and multiply by 2 since incoming is from -100 to 100.
		serialData[2] = (unsigned char)round(abs(2*(msg->linear.x)));
		// convert sign of power into digit 1 or 2 for arduino comm.
		serialData[3] = (unsigned char)round(1.5-0.5*abs(msg->linear.y)/(msg->linear.y));
		// remove sign from incoming data, and multiply by 2 since incoming is from -100 to 100.
		serialData[4] = (unsigned char)round(abs(2*(msg->linear.y)));
	}

	// Send data to arduino!
	write(fileDescriptor, serialData , 6*sizeof(char));

}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "motorcomm");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("motor_power", 200, motorPowerCallback);


  // The serial setup is mostly inspired by this:
  // https://stackoverflow.com/questions/38533480/c-libserial-serial-connection-to-arduino
  // Which sets things like parity, stop byte, length, etc.

  // This sets up the serial communication to the arduino driver.
    fileDescriptor = open("/dev/ttyACM1", O_RDWR | O_NOCTTY); //open link to arudino


    struct termios newtio;
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

    // set to 8N1
    newtio.c_cflag &= ~PARENB;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8;

    newtio.c_iflag = IGNPAR;

    // output mode to
    //newtio.c_oflag = 0;
    newtio.c_oflag |= OPOST;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME] = 10; /* inter-character timer 1 sec */
    newtio.c_cc[VMIN] = 0; /* blocking read disabled  */

    tcflush(fileDescriptor, TCIFLUSH);
    if (tcsetattr(fileDescriptor, TCSANOW, &newtio)) {
        perror("could not set the serial settings!");
        return -99;
    }


  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
