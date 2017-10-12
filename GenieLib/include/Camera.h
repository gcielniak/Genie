#pragma once

#include "Feature.h"
#include <opencv2/opencv.hpp>
///
/// A GenICam Camera class.
///

namespace GenICam {

	///The base interface for GenICam cameras
	class CameraBase {
	protected:
		cv::Mat image;

	public:
		//Get the captured image
		virtual const cv::Mat& Image() {
			return image;
		}

		//initialise the device
		virtual void Init(int device_nr) = 0;

		//start acquisition
		virtual void Start() = 0;

		//stop acquisition
		virtual void Stop() = 0;

		//capture image
		virtual void Capture() = 0;

		//get image (sensor) timestamp
		virtual long long TimeStamp() = 0;

		//get camera name
		virtual const std::string Name() = 0;
	};

	///Generic GenICam camera with a set of standard features
	class Camera : public CameraBase, public StandardFeature {
	private:
		double t_start, t_now;

	protected:
		IMG cvb_image;

		///Load driver
		static IMG LoadDriver(std::string driver_name = "%CVB%/drivers/GenICam.vin") {
			static const size_t DRIVERPATHSIZE = 256;
			IMG image;

			char driverPath[DRIVERPATHSIZE] = { 0 };
			cvbbool_t success = TranslateFileName(driver_name.c_str(), driverPath, DRIVERPATHSIZE);
			if (!success)
				throw std::runtime_error("GenICam::Init, TranslateFileName failed.");

			success = LoadImageFile(driverPath, image);
			if (!success) {
				string message = "GenICam::Init, Error loading \"" + std::string(driverPath) + "\" driver!";
				throw std::runtime_error(message.c_str());
			}
			return image;
		}

	public:
		///Constructor
		Camera() : cvb_image(0), t_start(0.0) {
		}

		///Destructor
		~Camera() {
			if (cvb_image)
				ReleaseObject(cvb_image);
		}

		virtual void Init(int device_nr = 0) {
			cvb_image = LoadDriver();
			Update(cvb_image);
			if (device_nr)
				Port(device_nr);
			TriggerMode(false);
		}

		virtual void Start() {
			if (G2Grab(cvb_image) < 0)
				throw std::runtime_error("Camera::Start, G2Grab failed.");

			image = cv::Mat(cv::Size(ImageWidth(cvb_image), ImageHeight(cvb_image)), ImageType());
		}

		virtual void Stop() {
			if (G2Freeze(cvb_image, true) < 0)
				throw std::runtime_error("Camera::Stop, G2Freeze failed.");
		}

		virtual void Capture() {
			while (G2Wait(cvb_image) < 0);//TODO: introduce time_out

			void* ppixels = nullptr; intptr_t xInc = 0; intptr_t yInc = 0;
			GetLinearAccess(cvb_image, 0, &ppixels, &xInc, &yInc);
			image.data = (uchar*)ppixels;
			//swap colour channels so it corresponds to OpenCV BGR format
			if (image.type() == CV_8UC3)
				cv::cvtColor(image, image, CV_RGB2BGR);
		}

		virtual long long TimeStamp() {
			//calculate timestamp in nanoseconds
			G2GetGrabStatus(cvb_image, GRAB_INFO_CMD::GRAB_INFO_TIMESTAMP, t_now);
			if (t_start == 0.0)
				t_start = t_now;
			long long timestamp = (long long)(t_now - t_start);
			return timestamp;
		}

		virtual const std::string Name() {
			return GetValueString("DeviceModelName");
		}

		///Change camera port 
		void Port(int index) {
			cvbbool_t success = CanCameraSelect2(cvb_image);
			if (!success)
				throw std::runtime_error("Camera::SetPort, CanCameraSelect2 failed.");
			
			IMG cvb_image_new = nullptr;
			cvbres_t error = CS2SetCamPort(cvb_image, index, 0, cvb_image_new);
			if (error < 0)
				throw std::runtime_error("Camera::SetPort, CS2SetCamPort failed.");
			ReleaseObject(cvb_image);
			cvb_image = cvb_image_new;

			Update(cvb_image);
		}

		///Get camera port
		int Port() {
			cvbval_t port;
			cvbres_t error = CS2GetCamPort(cvb_image, port);
			if (error < 0)
				throw std::runtime_error("Camera::GetPort, CS2GetCamPort failed.");

			return port;
		}

		///Get number of ports
		int PortNr() {
			cvbbool_t success = CanCameraSelect2(cvb_image);
			if (!success)
				throw std::runtime_error("Camera::PortNr, CanCameraSelect2 failed.");

			cvbval_t value = 0;

			cvbres_t error = CS2GetNumPorts(cvb_image, value);
			if (error < 0)
				throw std::runtime_error("Camera::PortNr, CS2GetNumPorts failed.");

			return value;
		}

		///Get image type
		int ImageType() {
			int type;
			if ((ImageDimension(cvb_image) == 3) && BitsPerPixel(ImageDatatype(cvb_image, 0) == 8))
				type = CV_8UC3;
			else if ((ImageDimension(cvb_image) == 1) && BitsPerPixel(ImageDatatype(cvb_image, 0) == 8))
				type = CV_8UC1;
			else if ((ImageDimension(cvb_image) == 1) && BitsPerPixel(ImageDatatype(cvb_image, 0) == 16))
				type = CV_16UC1;
			return type;
		}

		///Get all present devices supported by the driver
		///There can be multiple cameras
		void GetDevices(vector<string>& names) {
			int camera_index = 0;
			while (true) {
				try {
					Port(camera_index++);
					names.push_back(Name());
				}
				catch (std::runtime_error&) {
					return;
				}
			}
		}
	};

	///Dual camera providing RGB, NIR and NDVI images
	class RGBNCamera : public CameraBase {
	protected:
		bool trigger_software;
		GenICam::Camera rgb_camera, nir_camera;
		cv::Mat ndvi_image;
	
	public:
		RGBNCamera() : trigger_software(false) {
		}

		virtual void Init(int device_nr = 0) {
			rgb_camera.Init();
			nir_camera.Init();
		}

		virtual void Start() {
			rgb_camera.Start();
			nir_camera.Start();
		}

		virtual void Stop() {
			rgb_camera.Stop();
			nir_camera.Stop();
		}

		virtual void TriggerSoftwareMode(bool value) {
			trigger_software = value;
			rgb_camera.TriggerMode(value);
			nir_camera.TriggerMode(value);
		}

		virtual void Capture() {
			if (trigger_software)
				rgb_camera.TriggerSoftware();

			rgb_camera.Capture();
			nir_camera.Capture();
		}

		///Return 4-channel RGBN image by default
		virtual const cv::Mat& Image() {
			std::vector<cv::Mat> channels;
			cv::split(rgb_camera.Image(), channels); // break image into channels
			channels.push_back(nir_camera.Image());
			cv::merge(channels, image); // combine rgb and nir channels
			return image;
		}

		///Return RGB image
		const cv::Mat& RGBImage() {
			return rgb_camera.Image();
		}

		///Return NIR image
		const cv::Mat& NIRImage() {
			return nir_camera.Image();
		}

		///Return NDVI image
		const cv::Mat& NDVIImage() {
			//NDVI = (NIR - RED)/(NIR + RED)
			cv::Mat red, nir;
			std::vector<cv::Mat> rgb;
			cv::split(rgb_camera.Image(), rgb);
			rgb[0].convertTo(red, CV_32F);

			nir_camera.Image().convertTo(nir, CV_32F);

			ndvi_image = 255*((nir - red) / (nir + red) + 1.0) / 2;

			ndvi_image.convertTo(ndvi_image, CV_8UC1);

			return ndvi_image;
		}

		///Get image/sensor timestamp
		virtual long long TimeStamp() {
			return rgb_camera.TimeStamp();
		}

		///get timestamp for the rgb image
		long long RGBTimeStamp() {
			return rgb_camera.TimeStamp();
		}

		///get timestamp for the nir image
		long long NIRTimeStamp() {
			return nir_camera.TimeStamp();
		}

		///get handle to the RGB camera
		const GenICam::Camera& RGBCamera() {
			return rgb_camera;
		}

		///get handle to the NIR camera
		const GenICam::Camera& NIRCamera() {
			return nir_camera;
		}
	};
}
