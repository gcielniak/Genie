#pragma once

#include "Camera.h"

namespace TeledyneDALSA {
	class GenieNano : public GenICam::RGBNCamera {
		float dx, dy;
		bool software_trigger;

	public:
		GenieNano() : dx(0.0), dy(0.0) {
		}

		void Init(int device_nr = 0) {
			try {
				GenICam::RGBNCamera::Init();
			}
			catch (std::runtime_error&) {
				throw std::runtime_error("Could not initialise the Genie Nano camera.");
			}

			//assign the right port
			if (rgb_camera.Name() == "Nano-M2420")
				rgb_camera.Port(1);
			else if (nir_camera.Name() == "Nano-C2420")
				nir_camera.Port(1);

			if ((rgb_camera.Name() != "Nano-C2420") || (nir_camera.Name() != "Nano-M2420"))
				throw std::runtime_error("Could not initialise the Genie Nano camera.");
			if ((rgb_camera.ImageType() != CV_8UC3) && (nir_camera.ImageType() != CV_8UC1))
				throw std::runtime_error("Genie Nano camera found but with wrong image format.");

			AutoBrightness(true);
		}

		virtual void SoftwareTrigger(bool value) {
			GenICam::RGBNCamera::TriggerSoftwareMode(value);
			if (trigger_software) {
				rgb_camera.SetValue("LineSelector", 2);
				rgb_camera.SetValue("outputLineSource", 5);
			}
		}

		virtual const cv::Mat& Image() {
			cv::Mat M = cv::Mat::eye(2, 3, CV_32F);
			M.at<float>(0, 2) = dx;
			M.at<float>(1, 2) = dy;

			//apply affine transform
			cv::warpAffine(rgb_camera.Image(), image, M, rgb_camera.Image().size());

			std::vector<cv::Mat> channels;
			cv::split(image, channels); // break image into channels
			channels.push_back(nir_camera.Image());
			cv::merge(channels, image); // combine rgb and nir channels
			return image;
		}

		void SetTrans(float dx_new, float dy_new) {
			dx = dx_new;
			dy = dy_new;
		}

		void AutoBrightness(bool value) {
			rgb_camera.AutoBrightness(value);
			nir_camera.AutoBrightness(value);
		}
	};
}
