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
#include "amrl_msgs/HumanStateMsg.h"
#include "amrl_msgs/HumanStateArrayMsg.h"

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
using amrl_msgs::HumanStateMsg;
using amrl_msgs::HumanStateArrayMsg;

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
    human_publisher_ =
        n.advertise<HumanStateArrayMsg>(CONFIG_human_topic, 1, false);
    // Initial Extrinsics
    ext_translation_ = Vector3f(CONFIG_tx, CONFIG_ty, CONFIG_tz);
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    k4abt_tracker_create(&calibration_, tracker_config, &tracker);
  }

  Eigen::Vector3f GetCentroid(const k4abt_skeleton_t& skeleton) {
    Eigen::Vector3f centroid;
    for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++) {
      k4a_float3_t position = skeleton.joints[i].position;
      k4abt_joint_confidence_level_t confidence_level =
          skeleton.joints[i].confidence_level;
      Eigen::Vector3f joint_pose(position.v[0], position.v[1], position.v[2]);
      cout << "Pose: " << joint_pose.x() << ", " << joint_pose.y() << endl;
      cout << "Confidence: " << confidence_level << endl;
      if (confidence_level == K4ABT_JOINT_CONFIDENCE_MEDIUM) {
          centroid += joint_pose;
      }
    }
    return centroid / (int)K4ABT_JOINT_COUNT;
  }

  void GetHumans(const k4abt_frame_t& body_frame) {
      const size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
      humans_.human_states.clear();
      HumanStateMsg human;
      for (size_t i = 0; i < num_bodies; i++) {
          k4abt_skeleton_t skeleton;
          k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
          uint32_t id = k4abt_frame_get_body_id(body_frame, i);
          const Eigen::Vector3f centroid = GetCentroid(skeleton);

          human.id = id;
          human.pose.x = centroid.x();
          human.pose.y = centroid.y();
          humans_.human_states.push_back(human);
      }
  }

  void SkeletonCallback(k4a_capture_t capture) {
      // Queue the result into the tracker
      k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(
              tracker,
              capture,
              K4A_WAIT_INFINITE);
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
          GetHumans(body_frame);
          human_publisher_.publish(humans_);
          // detected
          printf("%zu bodies are detected!\n", num_bodies);
          // Remember to release the body frame once you finish using it
          k4abt_frame_release(body_frame);
      } else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT) { //  It should never hit timeout when K4A_WAIT_INFINITE is set.  printf("Error! Pop body frame result timeout!\n");
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
  ros::Publisher human_publisher_;
  image_transport::ImageTransport image_transport_;
  // Translation component of extrinsics.
  Eigen::Vector3f ext_translation_;
  HumanStateArrayMsg humans_;
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



