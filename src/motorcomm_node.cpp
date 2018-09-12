#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

geometry_msgs::Twist vel_msg;

void motorPowerCallback(const geometry_msgs::Twist::ConstPtr & msg)
{
	vel_msg = *msg; // This is how one can modify the data coming in.
	vel_msg.linear.x = 666;

	// Example on how to access the data, both from msg and vel_msg. Note the arrow.
	// It seems msg->anglular.z is essentially (*msg).angular.z
	// Note: Using the arrow the data is read only, which is normally what you need.
	// Using the pointers this way the data is never copied around, which is good for 
	// performance on the pi!
	ROS_INFO("I got this:\n Linear:\nx:%f\ny:%f\nz:%f\n MegaAngle:\nx:%f\ny:%f\nz:%f\n",
		vel_msg.linear.x, vel_msg.linear.y,msg->linear.z,
		msg->angular.x, msg->angular.y, msg->angular.z);


/* To test in ros, first run this node, then issue the command:
* rostopic pub /motor_power geometry_msgs/Twist "linear:
  x: 1.0
  y: 2.0
  z: 3.0
angular:
  x: 4.0
  y: 5.0
  z: 1.0" -1

*/
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

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
