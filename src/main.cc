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
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"

#include "math/geometry.h"
#include "k4a_wrapper.h"
#include "util/timer.h"

using Eigen::Vector3f;
using k4a_wrapper::K4AWrapper;

DECLARE_int32(v);
DEFINE_string(serial, "", "Serial number of kinect to open");
DEFINE_string(depth_topic, "kinect_depth", "Topic name for depth images");
DEFINE_string(points_topic, "kinect_points", "Topic name for point cloud");
DEFINE_string(frame_id, "kinect", "Frame ID");
DEFINE_bool(depth, true, "Publish depth images");
DEFINE_bool(points, true, "Publish point cloud");

class K4ARosInterface : public K4AWrapper {
 public:

  K4ARosInterface(
      ros::NodeHandle& n, 
      const std::string& serial,
      const k4a_device_configuration_t& config)  :
      K4AWrapper(serial, config, true) { 
    image_publisher_ = 
        n.advertise<sensor_msgs::Image>(FLAGS_depth_topic, 1, false);
    cloud_publisher_ = 
        n.advertise<sensor_msgs::PointCloud2>(FLAGS_points_topic, 1, false);
    InitMessages();
    InitLookups();
  }

  void InitMessages() {
    image_msg_.header.seq = 0;
    image_msg_.header.frame_id = FLAGS_frame_id;
    cloud_msg_.header = image_msg_.header;
    image_msg_.encoding = sensor_msgs::image_encodings::MONO16;
    image_msg_.is_bigendian = false;
    int width = calibration_.depth_camera_calibration.resolution_width;
    int height = calibration_.depth_camera_calibration.resolution_height;
    image_msg_.width = cloud_msg_.width = width;
    image_msg_.height = cloud_msg_.height = height;
    image_msg_.data.resize(width * height * sizeof(uint16_t));
    image_msg_.step = width * sizeof(uint16_t);

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
  }

  void InitLookups() {
    int width = calibration_.depth_camera_calibration.resolution_width;
    int height = calibration_.depth_camera_calibration.resolution_height;

    depth_ray_lookup_.resize(width * height);
    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;
    for (int y = 0, idx = 0; y < height; y++) {
      p.xy.y = (float)y;
      for (int x = 0; x < width; x++, idx++) {
        p.xy.x = (float)x;
        k4a_calibration_2d_to_3d(&calibration_, &p, 1.f, 
            K4A_CALIBRATION_TYPE_DEPTH, 
            K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);
        if (valid) {
            depth_ray_lookup_[idx] = 0.001 * Vector3f(ray.xyz.x, ray.xyz.y, 1);
        } else {
            depth_ray_lookup_[idx].setConstant(nanf(""));
        }
      }
    }

    width = calibration_.color_camera_calibration.resolution_width;
    height = calibration_.color_camera_calibration.resolution_height;

    rgbd_ray_lookup_.resize(width * height);
    for (int y = 0, idx = 0; y < height; y++) {
      p.xy.y = (float)y;
      for (int x = 0; x < width; x++, idx++) {
        p.xy.x = (float)x;
        k4a_calibration_2d_to_3d(
            &calibration_, 
            &p,
            1.f, 
            K4A_CALIBRATION_TYPE_COLOR, 
            K4A_CALIBRATION_TYPE_COLOR, 
            &ray, 
            &valid);
        if (valid) {
            rgbd_ray_lookup_[idx] = 0.001 * Vector3f(ray.xyz.x, ray.xyz.y, 1);
        } else {
            rgbd_ray_lookup_[idx].setConstant(nanf(""));
        }
      }
    }
    
  }

  void PublishPointCloud(k4a_image_t depth_image) {
    uint16_t* depth_data = 
        reinterpret_cast<uint16_t*>(k4a_image_get_buffer(depth_image));
    const int num_pixels = 
        calibration_.depth_camera_calibration.resolution_height * 
        calibration_.depth_camera_calibration.resolution_width;
    CHECK_EQ(num_pixels, static_cast<int>(depth_ray_lookup_.size()));
    Vector3f p;
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg_, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg_, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_rgb(cloud_msg_, "rgb");
    static uint32_t color = 0;
    for (int idx = 0; idx < num_pixels; ++idx) {
      p = static_cast<float>(depth_data[idx]) * depth_ray_lookup_[idx];
      // printf("%8.3f %8.3f %8.3f\n", p.x(), p.y(), p.z());
      *iter_x = p.z();
      *iter_y = -p.x();
      *iter_z = -p.y();
      *iter_rgb = color;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_rgb;
    }
    color += 0x10;
    printf("%08X\n", color);
    cloud_publisher_.publish(cloud_msg_);
  }

  void PublishDepth(k4a_image_t depth_image) {    
    memcpy(image_msg_.data.data(), k4a_image_get_buffer(depth_image), 
        image_msg_.data.size());
    image_publisher_.publish(image_msg_);
  }

  void DepthCallback(k4a_image_t image) override {
    if (FLAGS_v > 0) {
      printf("Received a frame, t=%f\n", GetMonotonicTime());
    }
    if (FLAGS_depth) PublishDepth(image);
    if (FLAGS_points) PublishPointCloud(image);
  }

  void RegisteredRGBDCallback(
        k4a_image_t color_image, k4a_image_t depth_image) override {
    if (FLAGS_v > 0) {
      printf("Received a registered frame, t=%f\n", GetMonotonicTime());
    }
    const int width = calibration_.color_camera_calibration.resolution_width;
    const int height = calibration_.color_camera_calibration.resolution_height;
    uint16_t* depth_data = 
        reinterpret_cast<uint16_t*>(k4a_image_get_buffer(depth_image));
    uint32_t* rgb_data = 
        reinterpret_cast<uint32_t*>(k4a_image_get_buffer(color_image));
    const int num_pixels = width * height;
    CHECK_EQ(num_pixels, static_cast<int>(rgbd_ray_lookup_.size()));
    Vector3f p;
    cloud_msg_.data.resize(num_pixels * cloud_msg_.point_step);
    cloud_msg_.width = width;
    cloud_msg_.height = height;
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg_, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg_, "z");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgb(cloud_msg_, "rgb");
    
    for (int idx = 0; idx < num_pixels; ++idx) {
      p = static_cast<float>(depth_data[idx]) * rgbd_ray_lookup_[idx];
      // printf("%8.3f %8.3f %8.3f\n", p.x(), p.y(), p.z());
      *iter_x = p.z();
      *iter_y = -p.x();
      *iter_z = -p.y();
      *iter_rgb = rgb_data[idx];;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_rgb;
    }
    cloud_publisher_.publish(cloud_msg_);
  }

  void UnregisteredRGBDCallback(
        k4a_image_t color_image, k4a_image_t depth_image) override {
    if (FLAGS_v > 0) {
      printf("Received an unregistered frame, t=%f\n", GetMonotonicTime());
    }
  }

 private:
  std::vector<Eigen::Vector3f> depth_ray_lookup_;
  std::vector<Eigen::Vector3f> rgbd_ray_lookup_;
  sensor_msgs::Image image_msg_;
  sensor_msgs::PointCloud2 cloud_msg_;
  ros::Publisher image_publisher_;
  ros::Publisher cloud_publisher_;
};

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  ros::init(argc, argv, "joystick");
  ros::NodeHandle n;
  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config.color_resolution = K4A_COLOR_RESOLUTION_720P;
  config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
  config.synchronized_images_only = true;
  K4ARosInterface interface(n, FLAGS_serial, config);

  while (ros::ok()) {
    interface.Capture();
    ros::spinOnce();
  }
  return 0;
}



