// Copyright (c) 2020 Joydeep Biswas joydeepb@cs.utexas.edu

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include <stdio.h>
#include <stdlib.h>
#include <k4a/k4a.hpp>

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "k4a/k4a.h"

#include "math/geometry.h"
#include "util/timer.h"

#include "k4a_wrapper.h"

using Eigen::Vector3f;

namespace k4a_wrapper {

K4AWrapper::K4AWrapper(
    const std::string& serial, 
    const k4a_device_configuration_t& config,
    bool enable_image_registration) : 
        device_(nullptr), 
        register_images_(enable_image_registration),
        config_(config) {
  OpenDevice(serial);
  k4a_device_get_calibration(
      device_, config_.depth_mode, config_.color_resolution, &calibration_);
  transformation_ = k4a_transformation_create(&calibration_);
  // Try to start the camera.
  if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device_, &config_)) {
    k4a_device_close(device_);
    abort();
  }
  if (K4A_RESULT_SUCCEEDED != k4a_device_start_imu(device_)) {
    k4a_device_close(device_);
    abort();
  }
}

K4AWrapper::~K4AWrapper() {
  k4a_device_close(device_);
}

std::string K4AWrapper::GetKinectSerial() {
  size_t n = 0;
  k4a_device_get_serialnum(device_, nullptr, &n);
  
  char serial_str[n];
  CHECK_EQ(k4a_device_get_serialnum(device_, serial_str, &n), 
      K4A_BUFFER_RESULT_SUCCEEDED);
  return std::string(serial_str);
}

void K4AWrapper::OpenDevice(const std::string& serial) {
  if (serial.empty()) {
    CHECK_EQ(k4a_device_open(0, &device_), K4A_RESULT_SUCCEEDED);
    printf("Opened Kinect with serial %s\n", GetKinectSerial().c_str());
    return;
  }
  const int n = k4a_device_get_installed_count();
  for (int i = 0; i < n; ++i) {
    if (k4a_device_open(0, &device_) != K4A_RESULT_SUCCEEDED) {
      // Could not open this device, it may already be in use, so skip.
      device_ = nullptr;
      continue;
    }
    if (serial == GetKinectSerial()) return;
    k4a_device_close(device_);
  }
  LOG(FATAL) << "Unable to find Kinect device with serial "
                << serial << ", aborting.\n";
  exit(1);
}

void K4AWrapper::Capture() {
  k4a_capture_t capture = NULL;
  switch (k4a_device_get_capture(device_, &capture, K4A_WAIT_INFINITE)) {
    case K4A_WAIT_RESULT_SUCCEEDED: {
      if (config_.depth_mode == K4A_DEPTH_MODE_OFF && 
          config_.color_resolution != K4A_COLOR_RESOLUTION_OFF) {
        // Only color.
        k4a_image_t color_image = k4a_capture_get_color_image(capture);
        ColorCallback(color_image);
        k4a_image_release(color_image);
      } else if (config_.depth_mode != K4A_DEPTH_MODE_OFF && 
                 config_.color_resolution == K4A_COLOR_RESOLUTION_OFF) {
        // Only depth.
        k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
        DepthCallback(depth_image);
        k4a_image_release(depth_image);
      } else if (config_.depth_mode != K4A_DEPTH_MODE_OFF && 
                 config_.color_resolution != K4A_COLOR_RESOLUTION_OFF) {
        // Color and depth.
        k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
        k4a_image_t color_image = k4a_capture_get_color_image(capture);
        if (depth_image == nullptr || color_image == nullptr) {
          // One of the frames was not received, drop this capture.
          if (FLAGS_v > 0) {
            fprintf(stderr, 
                    "Received at least one invalid frame, "
                    "depth:0x%08lX, "
                    "color:0x%08lX\n",
                    reinterpret_cast<uint64_t>(depth_image),
                    reinterpret_cast<uint64_t>(color_image));
          }
        } else if (register_images_) {
          k4a_image_t registered_depth;
          CHECK_EQ(k4a_image_create(
              K4A_IMAGE_FORMAT_DEPTH16,
              calibration_.color_camera_calibration.resolution_width,
              calibration_.color_camera_calibration.resolution_height,
              0,
              &registered_depth), K4A_RESULT_SUCCEEDED);
          CHECK_EQ(k4a_transformation_depth_image_to_color_camera(
              transformation_,
              depth_image,
              registered_depth), K4A_RESULT_SUCCEEDED);
          RegisteredRGBDCallback(color_image, registered_depth);
          k4a_image_release(registered_depth);
        } else {
          UnregisteredRGBDCallback(color_image, depth_image);
        }
        if (depth_image) k4a_image_release(depth_image);
        if (color_image) k4a_image_release(color_image);
      }
      k4a_capture_release(capture);

      // Process IMU sample queue
      k4a_imu_sample_t imu_sample;
      while (k4a_device_get_imu_sample(device_, &imu_sample, 0) ==
             K4A_WAIT_RESULT_SUCCEEDED) {
        ImuCallback(imu_sample);
      }
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

void DepthCallback(k4a_capture_t capture) {
  if (FLAGS_v > 1) {
    printf("Received a frame, t=%f\n", GetMonotonicTime());
  }
  k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
  if (depth_image == 0) {
    printf("Failed to get depth image from capture\n");
    exit(1);
  }
}


}  // namespace k4a_wrapper
