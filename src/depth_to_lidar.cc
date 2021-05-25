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

#ifdef CUDA_ENABLED
#include "processing_kernels.h"
#endif
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
DEFINE_bool(depth, false, "Publish depth images");
DEFINE_bool(points, false, "Publish point cloud");
DEFINE_bool(rgb, false, "Publish color images");
DEFINE_string(config_file, "config/kinect.lua", "Name of config file to use");

CONFIG_STRING(serial, "kinect_serial");
CONFIG_STRING(costmap_topic, "costmap_topic");
CONFIG_STRING(points_topic, "points_topic");
CONFIG_STRING(rgb_topic, "rgb_image_topic");
CONFIG_STRING(depth_topic, "depth_image_topic");
CONFIG_STRING(rgb_frame, "rgb_image_frame");
CONFIG_STRING(depth_frame, "depth_image_frame");
CONFIG_STRING(scan_topic, "scan_topic");
CONFIG_STRING(scan_frame, "scan_frame");
CONFIG_BOOL(registered, "registered_rgbd");

CONFIG_FLOAT(yaw, "rotation.yaw");
CONFIG_FLOAT(pitch, "rotation.pitch");
CONFIG_FLOAT(roll, "rotation.roll");
CONFIG_FLOAT(tx, "translation.x");
CONFIG_FLOAT(ty, "translation.y");
CONFIG_FLOAT(tz, "translation.z");
CONFIG_UINT(skip_points, "skip_points");
CONFIG_UINT(num_ranges, "num_ranges");

CONFIG_FLOAT(ground_angle_thresh, "ground_angle_thresh");
CONFIG_FLOAT(ground_dist_thresh, "ground_dist_thresh");
CONFIG_FLOAT(camera_angle_thresh, "camera_angle_thresh");
CONFIG_FLOAT(min_dist_thresh, "min_dist_thresh");

class DepthToLidar : public K4AWrapper {
 public:

  DepthToLidar(
      ros::NodeHandle& n, 
      const std::string& serial,
      const k4a_device_configuration_t& config)  :
      K4AWrapper(serial, config, CONFIG_registered),
      image_transport_(n) {
    costmap_publisher_ = 
        n.advertise<sensor_msgs::Image>(CONFIG_costmap_topic, 1, false);
    cloud_publisher_ = 
        n.advertise<sensor_msgs::PointCloud2>(CONFIG_points_topic, 1, false);
    scan_publisher_ = 
        n.advertise<sensor_msgs::LaserScan>(CONFIG_scan_topic, 1, false);
    rgb_publisher_ = image_transport_.advertise(CONFIG_rgb_topic, 1);
    depth_publisher_ = image_transport_.advertise(CONFIG_depth_topic, 1);
    InitMessages();
    InitLookups();
  }

  void InitMessages() {
    depth_msg_.header.seq = heightmap_msg_.header.seq = rgb_msg_.header.seq = 
        cloud_msg_.header.seq = scan_msg_.header.seq = 0;
    
    rgb_msg_.header.frame_id = CONFIG_rgb_frame;
    depth_msg_.header.frame_id = CONFIG_depth_frame;
    scan_msg_.header.frame_id = CONFIG_scan_frame;
    cloud_msg_.header.frame_id = CONFIG_scan_frame;

    heightmap_msg_.header = scan_msg_.header;
    // OpenCV Image format, float, 2 channel.
    heightmap_msg_.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
    heightmap_msg_.is_bigendian = false;

    rgb_msg_.encoding = sensor_msgs::image_encodings::BGRA8;
    rgb_msg_.is_bigendian = false;
    rgb_msg_.width = 
        calibration_.color_camera_calibration.resolution_width;
    rgb_msg_.height = 
        calibration_.color_camera_calibration.resolution_height;
    rgb_msg_.step = rgb_msg_.width * sizeof(uint32_t);
    rgb_msg_.data.resize(rgb_msg_.step * rgb_msg_.height);

    depth_msg_.encoding = sensor_msgs::image_encodings::MONO16;
    depth_msg_.is_bigendian = false;
    const int width = CONFIG_registered ?
        calibration_.color_camera_calibration.resolution_width : 
        calibration_.depth_camera_calibration.resolution_width;
    const int height = CONFIG_registered ?
        calibration_.color_camera_calibration.resolution_height :
        calibration_.depth_camera_calibration.resolution_height;
    depth_msg_.width = width;
    depth_msg_.height = height;
    depth_msg_.step = depth_msg_.width * sizeof(uint16_t);
    depth_msg_.data.resize(depth_msg_.step * depth_msg_.height);

    cloud_msg_.fields.resize(4);
    cloud_msg_.point_step = 3 * sizeof(float) + sizeof(uint32_t);
    cloud_msg_.is_dense = false;
    cloud_msg_.is_bigendian = false;
    cloud_msg_.fields[0].name = "x";
    cloud_msg_.fields[0].offset = 0;
    cloud_msg_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg_.fields[0].count = 1;
    cloud_msg_.fields[1].name = "y";
    cloud_msg_.fields[1].offset = 4;
    cloud_msg_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg_.fields[1].count = 1;
    cloud_msg_.fields[2].name = "z";
    cloud_msg_.fields[2].offset = 8;
    cloud_msg_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg_.fields[2].count = 1;
    cloud_msg_.fields[3].name = "rgb";
    cloud_msg_.fields[3].offset = 12;
    cloud_msg_.fields[3].datatype = sensor_msgs::PointField::UINT32;
    cloud_msg_.fields[3].count = 1;
    cloud_msg_.data.resize(width * height * cloud_msg_.point_step);
    cloud_msg_.width = width;
    cloud_msg_.height = height;
  }

  void InitLookups() {
    const int width = CONFIG_registered ?
        calibration_.color_camera_calibration.resolution_width : 
        calibration_.depth_camera_calibration.resolution_width;
    const int height = CONFIG_registered ?
        calibration_.color_camera_calibration.resolution_height :
        calibration_.depth_camera_calibration.resolution_height;
    const int num_pixels = width * height;
    points_.resize(num_pixels);
    colors_.resize(num_pixels);
    rgbd_ray_lookup_.resize(num_pixels);
    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;
    ext_translation_ = Vector3f(CONFIG_tx, CONFIG_ty, CONFIG_tz);
    const Matrix3f rotation =
        Matrix3f(AngleAxisf(DegToRad(CONFIG_yaw), Vector3f(0, 0, 1))) *
        Matrix3f(AngleAxisf(DegToRad(CONFIG_pitch), Vector3f(0, 1, 0))) *
        Matrix3f(AngleAxisf(DegToRad(CONFIG_roll), Vector3f(1, 0, 0)));
    
    for (int y = 0, idx = 0; y < height; y++) {
      p.xy.y = (float)y;
      for (int x = 0; x < width; x++, idx++) {
        p.xy.x = (float)x;
        if (CONFIG_registered) {
          k4a_calibration_2d_to_3d(
            &calibration_, 
            &p,
            1.f, 
            K4A_CALIBRATION_TYPE_COLOR, 
            K4A_CALIBRATION_TYPE_COLOR, 
            &ray, 
            &valid);
        } else {
          k4a_calibration_2d_to_3d(
            &calibration_, 
            &p,
            1.f, 
            K4A_CALIBRATION_TYPE_DEPTH, 
            K4A_CALIBRATION_TYPE_DEPTH, 
            &ray, 
            &valid);
        }
        if (valid) {
            rgbd_ray_lookup_[idx] = 
                0.001 * rotation * Vector3f(1, -ray.xyz.x, -ray.xyz.y);
        } else {
            rgbd_ray_lookup_[idx].setConstant(nanf(""));
        }
      }
    }
  }

  void PublishPointCloud() {
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg_, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg_, "z");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgb(cloud_msg_, "rgb");
    
    CHECK_EQ(points_.size(), colors_.size());
    for (size_t idx = 0; idx < points_.size(); ++idx) {
      *iter_x = points_[idx].x();
      *iter_y = points_[idx].y();
      *iter_z = points_[idx].z();
      *iter_rgb = colors_[idx];;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_rgb;
    }
    cloud_publisher_.publish(cloud_msg_);
  }

  void DepthToPointCloud(k4a_image_t color_image, k4a_image_t depth_image) {
    static CumulativeFunctionTimer ft(__FUNCTION__);
    CumulativeFunctionTimer::Invocation invoke(&ft);
    uint32_t* rgb_data = nullptr;
    if (CONFIG_registered && color_image != nullptr) {
        rgb_data = 
            reinterpret_cast<uint32_t*>(k4a_image_get_buffer(color_image));
    }
    uint16_t* depth_data = 
        reinterpret_cast<uint16_t*>(k4a_image_get_buffer(depth_image));
    for (size_t i = 0; i < points_.size(); ++i) {
      points_[i] = ext_translation_ + 
          (static_cast<float>(depth_data[i]) * rgbd_ray_lookup_[i]);
      if (rgb_data) {
        colors_[i] = rgb_data[i];
      } else {
        colors_[i] = 0xC0C0C0LU;
      }
    }
  }

  void PublishScan() {
    static CumulativeFunctionTimer ft(__FUNCTION__);
    CumulativeFunctionTimer::Invocation invoke(&ft);
    const float tan_a = tan(DegToRad(CONFIG_ground_angle_thresh));
    const float tan_ca = tan(DegToRad(CONFIG_camera_angle_thresh));
    const float angle_min = -M_PI_2;
    const float angle_max = M_PI_2;
    const int num_ranges = CONFIG_num_ranges;
    const float angle_increment = (angle_max - angle_min) / num_ranges;
    scan_msg_.ranges.assign(num_ranges, FLT_MAX);
    const int incr = 1 + CONFIG_skip_points;
    for (size_t i = 0; i < points_.size(); i += incr) {
      const Vector3f& p = points_[i];
      if (fabs(p.z() / p.x()) < tan_a || 
          fabs(p.z()) < CONFIG_ground_dist_thresh ||
          fabs((p.z() - CONFIG_tz) / p.x()) > tan_ca) continue;
      const float a = atan2(p.y(), p.x());
      const float r = Vector2f(p.x(), p.y()).norm();
      if (r <= CONFIG_min_dist_thresh) continue;
      const int index = (a - angle_min) / angle_increment;
      if (index < 0 || index >= num_ranges) continue;
      scan_msg_.ranges[index] = min(scan_msg_.ranges[index], r);
    }
    scan_msg_.angle_min = angle_min;
    scan_msg_.angle_max = angle_max;
    scan_msg_.angle_increment = angle_increment;
    scan_msg_.range_min = 0.0;
    scan_msg_.range_max = 10.0;
    scan_msg_.header.stamp = ros::Time::now();
    scan_publisher_.publish(scan_msg_);
  }

  void PublishRGBImage(k4a_image_t color_image) {
    static CumulativeFunctionTimer ft(__FUNCTION__);
    CumulativeFunctionTimer::Invocation invoke(&ft);
    uint32_t* rgb_data = 
          reinterpret_cast<uint32_t*>(k4a_image_get_buffer(color_image));
    memcpy(rgb_msg_.data.data(), rgb_data, rgb_msg_.data.size());
    rgb_msg_.header.stamp = ros::Time::now();
    rgb_publisher_.publish(rgb_msg_);
  }

  void PublishDepthImage(k4a_image_t depth_image) {
    static CumulativeFunctionTimer ft(__FUNCTION__);
    CumulativeFunctionTimer::Invocation invoke(&ft);
    uint16_t* depth_data = 
        reinterpret_cast<uint16_t*>(k4a_image_get_buffer(depth_image));
    memcpy(depth_msg_.data.data(), depth_data, depth_msg_.data.size());
    depth_msg_.header.stamp = ros::Time::now();
    depth_publisher_.publish(depth_msg_);
  }

  void PublishHeightMap() {
  }

  void RGBDCallback(k4a_image_t color_image, k4a_image_t depth_image) {
    if (color_image != nullptr && FLAGS_rgb) {
      PublishRGBImage(color_image);
    }

    if (depth_image == nullptr) return;
    DepthToPointCloud(color_image, depth_image);
    PublishScan();
    if (FLAGS_depth) {
      PublishDepthImage(depth_image);
    }
    if (FLAGS_points) {
      PublishPointCloud();
    }
  }

  void RegisteredRGBDCallback(k4a_image_t color_image, 
                              k4a_image_t depth_image) override {
    if (FLAGS_v > 1) {
      printf("Received a registered frame, t=%f\n", GetMonotonicTime());
    }
    RGBDCallback(color_image, depth_image);
  }

  void UnregisteredRGBDCallback(k4a_image_t color_image, 
                                k4a_image_t depth_image) override {
    if (FLAGS_v > 1) {
      printf("Received an unregistered frame, t=%f\n", GetMonotonicTime());
    }
    RGBDCallback(color_image, depth_image);
  }

 private:
  std::vector<Eigen::Vector3f> rgbd_ray_lookup_;
  std::vector<Eigen::Vector3f> points_;
  std::vector<uint32_t> colors_;
  sensor_msgs::LaserScan scan_msg_;
  sensor_msgs::Image rgb_msg_;
  sensor_msgs::Image depth_msg_;
  sensor_msgs::Image heightmap_msg_;
  sensor_msgs::PointCloud2 cloud_msg_;
  ros::Publisher costmap_publisher_;
  ros::Publisher cloud_publisher_;
  ros::Publisher scan_publisher_;
  image_transport::Publisher rgb_publisher_;
  image_transport::Publisher depth_publisher_;
  image_transport::Publisher heightmap_publisher_;
  image_transport::ImageTransport image_transport_;
  // Translation component of extrinsics.
  Eigen::Vector3f ext_translation_;
};

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  config_reader::ConfigReader reader({FLAGS_config_file});
  ros::init(argc, argv, "k4a_ros");
  ros::NodeHandle n;
  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config.color_resolution = K4A_COLOR_RESOLUTION_720P;
  config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
  config.synchronized_images_only = false;
  DepthToLidar interface(n, CONFIG_serial, config);

  while (ros::ok()) {
    interface.Capture();
    ros::spinOnce();
  }
  return 0;
}



