#include <iostream>
#include <string>
#include <AD_130GE.h>
#include <GenieNano.h>

int main(int argc, char* argv[]) {

	GenICam::JAI::AD_130GE camera;

	std::string window_name = "NDVI";
	int key = 0;

	try {
		camera.Init();
		camera.Start();
		cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
		while (key != 27) {

			camera.Capture();

			cv::Mat image = camera.NDVIImage();

			cv::imshow(window_name, image);

			key = cv::waitKey(1);
		}
	}
	catch (std::exception& e) {
		std::cerr << e.what() << std::endl;
	}


	return 0;
}
