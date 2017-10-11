#include "ros/ros.h"
#include <AD_130GE.h>
#include <sstream>

int main(int argc, char **argv) {

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  ros::Publisher image_pub = n.advertise<std_msgs::Image>("camera/image", 100);

  ros::Rate loop_rate(1);

  int count = 0;

  std_msgs::Image msg;

	JAI::AD_130GE camera;

	try {
		camera.Init();
	}
	catch (std::exception& e) {
		cerr << e.what() << endl;
}

  while (ros::ok()) {

	if (image_pub.getNumSubscribers()) {
	  std::stringstream ss;
	  ss << "hello world " << count;
	  msg.data = ss.str();

	    ROS_INFO("%s", msg.data.c_str());
	    image_pub.publish(msg);
	    loop_rate.sleep();
	    ++count;
	}

    ros::spinOnce();
  }

  return 0;
}

