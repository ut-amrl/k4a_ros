// Copyright (c) 2021 Jarrett holtz

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
#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "image_transport/image_transport.h"
#include "omp.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"

#include "config_reader/config_reader.h"
#include "math/geometry.h"
#include "k4a_wrapper.h"
#include "util/helpers.h"
#include "util/timer.h"

using std::string;
using Eigen::Affine3f;
using Eigen::AngleAxisf;
using Eigen::Matrix3f;
using Eigen::Translation3f;
using Eigen::Vector2f;
using Eigen::Vector3f;
using k4a_wrapper::K4AWrapper;
using std::max;
using std::min;
using std::vector;

using namespace math_util;

DECLARE_int32(v);
DEFINE_string(config_file, "config/kinect.lua", "Name of config file to use");

CONFIG_STRING(serial, "kinect_serial");
CONFIG_STRING(human_topic, "human_topic");
CONFIG_STRING(human_frame, "human_frame");

CONFIG_FLOAT(yaw, "rotation.yaw");
CONFIG_FLOAT(pitch, "rotation.pitch");
CONFIG_FLOAT(roll, "rotation.roll");
CONFIG_FLOAT(tx, "translation.x");
CONFIG_FLOAT(ty, "translation.y");
CONFIG_FLOAT(tz, "translation.z");

class SkeletonsToHumans: public K4AWrapper {
 public:


  SkeletonsToHumans(
      ros::NodeHandle& n,
      const std::string& serial,
      const k4a_device_configuration_t& config)  :
      K4AWrapper(serial, config, false),
      image_transport_(n) {
    // human_publisher_ =
        // n.advertise<sensor_msgs::LaserScan>(CONFIG_scan_topic, 1, false);
    InitMessages();
    // Initial Extrinsics
    ext_translation_ = Vector3f(CONFIG_tx, CONFIG_ty, CONFIG_tz);
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    k4abt_tracker_create(&calibration_, tracker_config, &tracker);
  }

  void SkeletonCallback(k4a_capture_t capture) {
      // Queue the result into the tracker
      k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(
              tracker,
              capture,
              K4A_WAIT_INFINITE);
      // Check and Release
      k4a_capture_release(capture);
      if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT) {
          // It should never hit timeout when K4A_WAIT_INFINITE is set.
          printf("Error! Add capture to tracker process queue timeout!\n");
          return;
      } else if (queue_capture_result == K4A_WAIT_RESULT_FAILED) {
          printf("Error! Add capture to tracker process queue failed!\n");
          return;
      }

      // Now we need to handle the result.
      k4abt_frame_t body_frame = NULL;
      k4a_wait_result_t pop_frame_result =
          k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
      if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED) {
          // Successfully popped the body tracking result. Start your processing
          size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
          // TODO(jaholtz) Figure out how to get centroid, etc.
          // TODO(jaholtz) fill the human states message for each human
          // detected
          printf("%zu bodies are detected!\n", num_bodies);
          // Remember to release the body frame once you finish using it
          k4abt_frame_release(body_frame);
      } else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT) {
          //  It should never hit timeout when K4A_WAIT_INFINITE is set.
          printf("Error! Pop body frame result timeout!\n");
          return;
      } else {
          printf("Pop body frame result failed!\n");
          return;
      }
  }

  void InitMessages() {
  }

 private:
  // Vector of Humans
  // Last skeleton detection
  ros::Publisher human_publisher_;
  image_transport::ImageTransport image_transport_;
  // Translation component of extrinsics.
  Eigen::Vector3f ext_translation_;
  k4abt_tracker_t tracker = NULL;
};

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  config_reader::ConfigReader reader({FLAGS_config_file});
  ros::init(argc, argv, "k4a_ros");
  ros::NodeHandle n;
  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  // Copy Configuration from Example
  config.color_resolution = K4A_COLOR_RESOLUTION_720P;
  config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
  config.synchronized_images_only = false;
  SkeletonsToHumans interface(n, CONFIG_serial, config);

  while (ros::ok()) {
    // TODO(jaholtz) add skeleton capture to the capture command
    interface.Capture();
    ros::spinOnce();
  }
  return 0;
}



