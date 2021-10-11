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
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

using std::string;
using Eigen::Affine3f;
using Eigen::AngleAxisf;
using Eigen::Matrix3f;
using Eigen::Translation3f;
using Eigen::Vector2f;
using Eigen::Vector3f;
using k4a_wrapper::K4AWrapper;
using std::max;
using std::map;
using std::min;
using std::vector;
using amrl_msgs::HumanStateMsg;
using amrl_msgs::HumanStateArrayMsg;
using visualization_msgs::MarkerArray;
using visualization_msgs::Marker;

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
CONFIG_FLOAT(max_track_length, "max_track_length");
CONFIG_FLOAT(age_threshold, "age_threshold");

struct Track {
    int id = -1;
    int age = 0;
    vector<double> times;
    vector<HumanStateMsg> observations;
    HumanStateMsg human;
};

class SkeletonsToHumans: public K4AWrapper {
 public:


  SkeletonsToHumans(
      ros::NodeHandle& n,
      const std::string& serial,
      const k4a_device_configuration_t& config)  :
      K4AWrapper(serial, config, false),
      image_transport_(n) {
    InitMessages();
    human_publisher_ =
        n.advertise<HumanStateArrayMsg>(CONFIG_human_topic, 1, false);
    marker_pub_ =
        n.advertise<MarkerArray>("visualization_marker_array", 1, false);
    // Initial Extrinsics
    ext_translation_ = Vector3f(CONFIG_tx, CONFIG_ty, CONFIG_tz);
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    k4abt_tracker_create(&calibration_, tracker_config, &tracker_);
  }

  Eigen::Vector3f GetCentroid(const k4abt_skeleton_t& skeleton) {
    Eigen::Vector3f centroid;
    int count = 0;
    for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++) {
      k4a_float3_t position = skeleton.joints[i].position;
      k4abt_joint_confidence_level_t confidence_level =
          skeleton.joints[i].confidence_level;
      Eigen::Vector3f joint_pose(position.v[0], position.v[1], position.v[2]);
      if (confidence_level == K4ABT_JOINT_CONFIDENCE_MEDIUM) {
          centroid += joint_pose;
          count++;
      }
    }
    return centroid / count;
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
          human.pose.x = centroid.z() / 1000.0;
          human.pose.y = -centroid.x() / 1000.0;
          humans_.human_states.push_back(human);
      }
  }

  void CreateTrackSimple(const HumanStateMsg& human) {
      Track track;
      track.id = human.id;
      track.age = 0;
      track.times.push_back(current_time_);
      track.observations.push_back(human);
      track.human = human;
      tracks_[track.id] = track;
  }

  void UpdateTrackSimple(const HumanStateMsg& human) {
      // Track is updated this frame.
      Track& track = tracks_[human.id];
      track.age = 0;
      track.observations.push_back(human);
      track.times.push_back(current_time_);
      track.human = human;
      if (track.observations.size() >= CONFIG_max_track_length) {
          track.observations.erase(track.observations.begin());
          track.times.erase(track.times.begin());
      }
  }

  void PruneTracks() {
      map<int, Track> new_tracks;
      for (auto& entry : tracks_) {
          entry.second.age++;
          if (entry.second.age <= CONFIG_age_threshold) {
              new_tracks[entry.first] = entry.second;
          }
      }
      tracks_ = new_tracks;
  }

  Vector2f GetVelocity(const Track& track) {
      Vector2f sum(0,0);
      for (size_t i = 1; i < track.times.size(); ++i) {
          const HumanStateMsg obs_1 = track.observations[i - 1];
          const HumanStateMsg obs_2 = track.observations[i];
          const double time_1 = track.times[i - 1];
          const double time_2 = track.times[i];
          const Vector2f pose_1(obs_1.pose.x, obs_1.pose.y);
          const Vector2f pose_2(obs_2.pose.x, obs_2.pose.y);

          const double time_diff = time_2 - time_1;
          const Vector2f vel = (pose_1 - pose_2) / time_diff;
          sum += vel;
      }
      // The average/mean velocity
      return sum / track.times.size();
  }

  void EstimateVelocity() {
      for (auto& entry : tracks_) {
          Track& track = entry.second;
          const Vector2f vel = GetVelocity(track);
          track.human.translational_velocity.x = vel[0];
          track.human.translational_velocity.y = vel[1];
      }
  }

  void PublishHumans() {
      HumanStateArrayMsg humans;
      for (auto& entry : tracks_) {
          humans.human_states.push_back(entry.second.human);
      }
      human_publisher_.publish(humans);
      humans_ = humans;
  }

  // TODO(jaholtz) test sdk tracking, use kalman filter if necessary
  void TrackHumans() {
      // For each human check if it matches the id of a previous human.
      for (const HumanStateMsg& human : humans_.human_states) {
          // If distance is too large (or matches no id) create a new track:
          // else, update the old one
          if (tracks_.find(human.id) != tracks_.end()) {
              UpdateTrackSimple(human);
          } else {
              CreateTrackSimple(human);
          }
      }

      // Prune tracks that have not been updated recently
      PruneTracks();

      // Given the tracks, calculate the current velocity
      EstimateVelocity();

      // Publish the Human Detections
      PublishHumans();

      // Visualize the Human Detections
  }

  void SkeletonCallback(k4a_capture_t capture) {
      // TODO(jaholtz) store the time of the observation and the last one
      current_time_ = ros::Time::now().toSec();
      // Queue the result into the tracker
      k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(
              tracker_,
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
          k4abt_tracker_pop_result(tracker_, &body_frame, K4A_WAIT_INFINITE);
      if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED) {
          // Successfully popped the body tracking result. Start your processing
          GetHumans(body_frame);
          // Remember to release the body frame once you finish using it
          k4abt_frame_release(body_frame);
      } else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT) {
          return;
      } else {
          printf("Pop body frame result failed!\n");
          return;
      }
      TrackHumans();
  }

  void InitMessages() {
  }

 private:
  // Vector of Humans
  ros::Publisher human_publisher_;
  ros::Publisher marker_pub_;
  image_transport::ImageTransport image_transport_;
  // Translation component of extrinsics.
  Eigen::Vector3f ext_translation_;
  HumanStateArrayMsg humans_;
  k4abt_tracker_t tracker_ = NULL;
  double current_time_;
  map<int, Track> tracks_;
};

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  config_reader::ConfigReader reader({FLAGS_config_file});
  ros::init(argc, argv, "k4a_ros");
  ros::NodeHandle n;
  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
  config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
  config.synchronized_images_only = false;
  SkeletonsToHumans interface(n, CONFIG_serial, config);

  while (ros::ok()) {
    interface.Capture();
    ros::spinOnce();
  }
  return 0;
}

