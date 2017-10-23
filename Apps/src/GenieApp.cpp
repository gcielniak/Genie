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

			cv::Mat rgb_image = camera.RGBImage();

			cv::Mat vis_image = rgb_image;
			std::vector<cv::Mat> channels;
			cv::split(rgb_image, channels); // break image into channels
			cv::merge({ { channels[2], channels[2], channels[2] } }, vis_image);

			cv::Mat nir_image = camera.NIRImage();
			cv::merge({ { nir_image, nir_image, nir_image} }, nir_image);

			cv::Mat ndvi_image = camera.NDVIImage();
			cv::merge({ { ndvi_image, ndvi_image, ndvi_image } }, ndvi_image);

			cv::Mat row_1, row_2, image;
			cv::hconcat(row_1, vis_image, nir_image);
			cv::hconcat(row_2, rgb_image, ndvi_image);
			cv::vconcat(image, row_1, row_2);

			cv::imshow(window_name, image);

			key = cv::waitKey(1);
		}
	}
	catch (std::exception& e) {
		std::cerr << e.what() << std::endl;
	}


	return 0;
}
