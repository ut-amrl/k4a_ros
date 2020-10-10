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
#include "k4a_image/K4ACaptureRecipient.h"
#include "k4a_image/KinectWrapper.h"
#include "util/timer.h"

using Eigen::Vector3f;

DECLARE_int32(v);
DEFINE_string(serial, "", "Serial number of kinect to open");
DEFINE_string(depth_topic, "kinect_depth", "Topic name for depth images");
DEFINE_string(points_topic, "kinect_points", "Topic name for point cloud");
DEFINE_string(frame_id, "kinect", "Frame ID");
DEFINE_bool(depth, true, "Publish depth images");
DEFINE_bool(points, true, "Publish point cloud");

class K4ARosInterface {
 public:
  ~K4ARosInterface() {
    k4a_device_close(device_);
  }

  std::string GetKinectSerial() {
    size_t n = 0;
    k4a_device_get_serialnum(device_, nullptr, &n);
    
    char serial_str[n];
    CHECK_EQ(k4a_device_get_serialnum(device_, serial_str, &n), 
        K4A_BUFFER_RESULT_SUCCEEDED);
    return std::string(serial_str);
  }

  void OpenDevice(const std::string& serial) {
    if (serial.empty()) {
      CHECK_EQ(k4a_device_open(0, &device_), K4A_RESULT_SUCCEEDED);
      printf("Opened Kinect with serial %s\n", GetKinectSerial().c_str());
      return;
    }
    const int n = k4a_device_get_installed_count();
    for (int i = 0; i < n; ++i) {
      CHECK_EQ(k4a_device_open(0, &device_), K4A_RESULT_SUCCEEDED);
      if (serial == GetKinectSerial()) return;
    }
    LOG(FATAL) << "Unable to find Kinect device with serial "
                 << serial << ", aborting.\n";
    exit(1);
  }

  K4ARosInterface(ros::NodeHandle& n, const std::string& serial)  :
      device_(nullptr) {
    OpenDevice(serial);

    config_ = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config_.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    // config_.synchronized_images_only = true;
    k4a_device_get_calibration(
        device_, config_.depth_mode, config_.color_resolution, &calibration_);

    // try to start cameras
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device_, &config_)) {
      k4a_device_close(device_);
      abort();
    }
    image_publisher_ = 
        n.advertise<sensor_msgs::Image>(FLAGS_depth_topic, 1, false);
    cloud_publisher_ = 
        n.advertise<sensor_msgs::PointCloud2>(FLAGS_points_topic, 1, false);
    image_msg_.header.seq = 0;
    image_msg_.header.frame_id = FLAGS_frame_id;
    cloud_msg_.header = image_msg_.header;
    image_msg_.encoding = sensor_msgs::image_encodings::MONO16;
    image_msg_.is_bigendian = false;

    InitLookups();
  }

  void Capture() {
    k4a_capture_t capture = NULL;
    switch (k4a_device_get_capture(device_, &capture, K4A_WAIT_INFINITE)) {
      case K4A_WAIT_RESULT_SUCCEEDED: {
        DepthCallback(capture);
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

  void InitLookups() {
    int width = calibration_.depth_camera_calibration.resolution_width;
    int height = calibration_.depth_camera_calibration.resolution_height;
    image_msg_.width = cloud_msg_.width = width;
    image_msg_.height = cloud_msg_.height = height;
    image_msg_.data.resize(width * height * sizeof(uint16_t));
    image_msg_.step = width * sizeof(uint16_t);

    cloud_msg_.fields.resize(3);
    cloud_msg_.point_step = 3 * sizeof(float);
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
    cloud_msg_.data.resize(width * height * 3 * sizeof(float));


    ray_lookup_.resize(width * height);
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
            ray_lookup_[idx] = 0.001 * Vector3f(ray.xyz.x, ray.xyz.y, 1);
        } else {
            ray_lookup_[idx].setConstant(nanf(""));
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
    CHECK_EQ(num_pixels, static_cast<int>(ray_lookup_.size()));
    Vector3f p;
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg_, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg_, "z");
    for (int idx = 0; idx < num_pixels; ++idx) {
      p = static_cast<float>(depth_data[idx]) * ray_lookup_[idx];
      // printf("%8.3f %8.3f %8.3f\n", p.x(), p.y(), p.z());
      *iter_x = p.z();
      *iter_y = -p.x();
      *iter_z = -p.y();
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    cloud_publisher_.publish(cloud_msg_);
  }

  void PublishDepth(k4a_image_t depth_image) {    
    memcpy(image_msg_.data.data(), k4a_image_get_buffer(depth_image), 
        image_msg_.data.size());
    image_publisher_.publish(image_msg_);
  }

  void DepthCallback(k4a_capture_t capture) {
    if (FLAGS_v > 0) {
      printf("Received a frame, t=%f\n", GetMonotonicTime());
    }
    k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
    if (depth_image == 0) {
      printf("Failed to get depth image from capture\n");
      exit(1);
    }
    if (FLAGS_depth) PublishDepth(depth_image);
    if (FLAGS_points) PublishPointCloud(depth_image);
  }
 private:
  sensor_msgs::Image image_msg_;
  sensor_msgs::PointCloud2 cloud_msg_;
  ros::Publisher image_publisher_;
  ros::Publisher cloud_publisher_;

  k4a_device_t device_;
  k4a_calibration_t calibration_;
  k4a_device_configuration_t config_;
  std::vector<Eigen::Vector3f> ray_lookup_;
};

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  ros::init(argc, argv, "joystick");
  ros::NodeHandle n;
  K4ARosInterface interface(n, FLAGS_serial);

  while (ros::ok()) {
    interface.Capture();
    ros::spinOnce();
  }
  return 0;
}



