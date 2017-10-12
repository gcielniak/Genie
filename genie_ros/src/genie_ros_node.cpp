#include <sstream>
#include <AD_130GE.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("camera/image", 100);

  ros::Rate loop_rate(1);

  int count = 0;

  sensor_msgs::Image msg;

	cv_bridge::CvImage img_bridge;

std_msgs::Header header; // empty header

	GenICam::JAI::AD_130GE camera;

	try {
		camera.Init();
		camera.Start();
	}
	catch (std::exception& e) {
		cerr << e.what() << endl;
	}

	cerr << "Camera Initialised." << endl;

  while (ros::ok()) {

	if (image_pub.getNumSubscribers()) {
		camera.Capture();
		header.seq = count; // user defined counter
		header.stamp = ros::Time::now(); // time
		img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, camera.NDVIImage());
		img_bridge.toImageMsg(msg); // from cv_bridge to sensor_msgs::Image
	    image_pub.publish(msg);
	    ++count;
	}

    ros::spinOnce();
  }

  return 0;
}

