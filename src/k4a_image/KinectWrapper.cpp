#include <stdio.h>

#include <cstdio>
#include <cstdlib>

#include "KinectWrapper.h"
#include "k4a/k4a.hpp"

KinectWrapper::KinectWrapper(uint8_t deviceIndex, K4ACaptureRecipient& kfr) :
    device_(nullptr), recipient_(kfr) {
  if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &device_)) {
    fprintf(stderr, "ERROR: Unable to open Kinect device, aborting.\n");
    return;
  }

  k4a_device_get_calibration(device_,
                             K4A_DEPTH_MODE_NFOV_UNBINNED,
                             K4A_COLOR_RESOLUTION_2160P, 
                             &calibration_);
  printf("found %d intrinsic params\n",
      calibration_.color_camera_calibration.intrinsics.parameter_count);
  printf("%f %f %f %f \n",
      calibration_.color_camera_calibration.intrinsics.parameters.param.cx,
      calibration_.color_camera_calibration.intrinsics.parameters.param.cy,
      calibration_.color_camera_calibration.intrinsics.parameters.param.fx,
      calibration_.color_camera_calibration.intrinsics.parameters.param.fy);

  config_ = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config_.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
  // config_.synchronized_images_only = true;

  // try to start cameras
  if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device_, &config_)) {
    k4a_device_close(device_);
    abort();
  }
}

KinectWrapper::~KinectWrapper() {
  k4a_device_close(device_);
}

void KinectWrapper::capture() {
  k4a_capture_t capture = NULL;
  switch (k4a_device_get_capture(device_, &capture, K4A_WAIT_INFINITE)) {
    case K4A_WAIT_RESULT_SUCCEEDED: {
      recipient_.receiveFrame(capture);
      k4a_capture_release(capture);
    } break;
    case K4A_WAIT_RESULT_TIMEOUT: {
      printf("Timed out waiting for a capture\n");
      k4a_device_close(device_);
    } break;
    case K4A_WAIT_RESULT_FAILED: {
        printf("Failed to read a capture\n");
        k4a_device_close(device_);
    } break;
    default: {
      fprintf(stderr, "ERROR: Unexpected response from Kinect capture\n");
    }
  }
}