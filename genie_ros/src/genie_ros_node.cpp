#include <sstream>
#include <AD_130GE.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "genie");

  ros::NodeHandle nh;
  ros::Publisher rgb_pub = nh.advertise<sensor_msgs::Image>("camera/rgb_image", 100);
  ros::Publisher nir_pub = nh.advertise<sensor_msgs::Image>("camera/nir_image", 100);
  ros::Publisher rgbn_pub = nh.advertise<sensor_msgs::Image>("camera/rgbn_image", 100);
  ros::Publisher ndvi_pub = nh.advertise<sensor_msgs::Image>("camera/ndvi_image", 100);
	
  int rgb_count = 0;
  int nir_count = 0;
  int rgbn_count = 0;
  int ndvi_count = 0;
	
  sensor_msgs::Image image_msg;
	cv_bridge::CvImage img_bridge;

	std_msgs::Header header; // empty header

	GenICam::JAI::AD_130GE camera;

	try {
		camera.Init();
		camera.Start();
	}
	catch (std::exception& e) {
		std::cerr << e.what() << std::endl;
		return 0;
	}

	cerr << "RGBN Camera Initialised." << endl;

  while (ros::ok()) {

		if (rgb_pub.getNumSubscribers() ||
		nir_pub.getNumSubscribers() ||
		rgbn_pub.getNumSubscribers() ||
		ndvi_pub.getNumSubscribers()) {
			camera.Capture();
			
			header.stamp = ros::Time::now(); // time

			//rgb images
			if (rgb_pub.getNumSubscribers()) {
				header.seq = rgb_count++;
				img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8,
					camera.RGBImage());
				img_bridge.toImageMsg(image_msg);
				rgb_pub.publish(image_msg);
			}

			//nir images
			if (nir_pub.getNumSubscribers()) {
				header.seq = nir_count++;
				img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8,
					camera.NIRImage());
				img_bridge.toImageMsg(image_msg);
				nir_pub.publish(image_msg);
			}

			//rgbn images
			if (rgbn_pub.getNumSubscribers()) {
				header.seq = rgbn_count++;
				img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGRA8,
					camera.Image());
				img_bridge.toImageMsg(image_msg);
				rgbn_pub.publish(image_msg);
			}

			//ndvi images
			if (ndvi_pub.getNumSubscribers()) {
				header.seq = ndvi_count++;
				img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8,
					camera.NDVIImage());
				img_bridge.toImageMsg(image_msg);
				ndvi_pub.publish(image_msg);
			}
		}

		ros::spinOnce();
  }

  return 0;
}

