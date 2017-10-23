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

			cv::Mat vis_image;
			std::vector<cv::Mat> channels;
			cv::split(rgb_image, channels); // break image into channels
			channels[0] = channels[2];
			channels[1] = channels[2];
			cv::merge(channels, vis_image);

			cv::Mat nir_image = camera.NIRImage();
			channels[0] = channels[1] = channels[2] = nir_image;
			cv::merge(channels, nir_image);

			cv::Mat ndvi_image = camera.NDVIImage();
			channels[0] = channels[1] = channels[2] = ndvi_image;
			cv::merge(channels, ndvi_image);

			cv::Mat row_1, row_2, image;
			cv::hconcat(vis_image, nir_image, row_1);
			cv::hconcat(rgb_image, ndvi_image, row_2);
			cv::vconcat(row_1, row_2, image);

			cv::resize(image, image, image.size() / 2);

			cv::imshow(window_name, image);

			key = cv::waitKey(1);
		}
	}
	catch (std::exception& e) {
		std::cerr << e.what() << std::endl;
	}


	return 0;
}
