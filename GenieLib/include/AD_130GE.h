#pragma once

#include "Camera.h"

namespace GenICam {
namespace JAI {

	enum ShutterModeType {
		ProgrammableExposure = 1,
		ExposureTimeAbs = 2,
		ExposureAutoContinuous = 3
	};

	///
	/// AD-130GE: dual chip, prism-based RGBN camera.
	/// more info: http://www.jai.com/en/products/ad-130ge
	///
	class AD_130GE : public GenICam::RGBNCamera {
	public:
		virtual void Init(int device_nr = 0) {
			try {
				GenICam::RGBNCamera::Init();
			}
			catch (std::runtime_error&) {
				throw std::runtime_error("Could not initialise the AD-130GE camera.");
			}
			
			//assign the right port
			if (rgb_camera.Name() == "AD-130GE_#1")
				rgb_camera.Port(1);
			else if (nir_camera.Name() == "AD-130GE_#0")
				nir_camera.Port(1);

			if ((rgb_camera.Name() != "AD-130GE_#0") || (nir_camera.Name() != "AD-130GE_#1"))
				throw std::runtime_error("Could not initialise the AD-130GE camera.");

			if ((rgb_camera.ImageType() != CV_8UC3) && (nir_camera.ImageType() != CV_8UC1))
				throw std::runtime_error("AD-130GE camera found but with the wrong image format.");

			//a fix that sets the rgb image to 966
			//requires another initialisation which needs to be investigated
			rgb_camera.Height(966);
			rgb_camera.Init();
			if (rgb_camera.Name() == "AD-130GE_#1")
				rgb_camera.Port(1);

			//enforce two camera sync
			SyncMode(true);

			//adjust auto-tune parameters
			GainAuto(true);
			ShutterMode(JAI::ExposureAutoContinuous);

			SoftwareTrigger(true);
		}

		void SoftwareTrigger(bool value) {
			GenICam::RGBNCamera::TriggerSoftwareMode(value);
			if (trigger_software) {
				rgb_camera.TriggerSource(GenICam::Software);
				nir_camera.TriggerSource(GenICam::Software);
			}
		}

		void SyncMode(bool value) {
			rgb_camera.SetValue("SyncMode", (int)(value ? 0 : 1));
		}

		bool SyncMode() {
			return (rgb_camera.GetValueInteger("SyncMode") == 0) ? true : false;
		}

		void ShutterMode(ShutterModeType type) {
			rgb_camera.SetValue("ShutterMode", type);
			nir_camera.SetValue("ShutterMode", type);
		}

		void GainAuto(bool value) {
			rgb_camera.GainAuto(value);
			nir_camera.GainAuto(value);
		}

		virtual const std::string Name() {
			return "AD-130GE";
		}
	};
}
}
