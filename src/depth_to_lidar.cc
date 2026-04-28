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
#include <exception>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "omp.h"
#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

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

namespace {
builtin_interfaces::msg::Time ToBuiltinTime(const rclcpp::Time& t) {
  builtin_interfaces::msg::Time out;
  const int64_t ns = t.nanoseconds();
  out.sec = static_cast<int32_t>(ns / 1000000000LL);
  out.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return out;
}
}  // namespace

DECLARE_int32(v);
DEFINE_bool(depth, false, "Publish depth images");
DEFINE_bool(points, false, "Publish point cloud");
DEFINE_bool(scan, false, "Publish laser scan");
DEFINE_bool(rgb, true, "Publish color images");
DEFINE_bool(imu, true, "Publish IMU data");
DEFINE_string(config_file, "config/kinect.lua", "Name of config file to use");
DEFINE_uint32(resolution, 720, "RGB Image Resolution");
DEFINE_uint32(fps, 15, "RGB image frame rate");

CONFIG_STRING(serial, "kinect_serial");
CONFIG_STRING(costmap_topic, "costmap_topic");
CONFIG_STRING(points_topic, "points_topic");
CONFIG_STRING(rgb_topic, "rgb_image_topic");
CONFIG_STRING(depth_topic, "depth_image_topic");
CONFIG_STRING(rgb_camera_info_topic, "rgb_camera_info_topic");
CONFIG_STRING(depth_camera_info_topic, "depth_camera_info_topic");
CONFIG_STRING(rgb_frame, "rgb_image_frame");
CONFIG_STRING(depth_frame, "depth_image_frame");
CONFIG_STRING(scan_topic, "scan_topic");
CONFIG_STRING(scan_frame, "scan_frame");
CONFIG_STRING(imu_topic, "imu_topic");
CONFIG_STRING(imu_frame, "imu_frame");
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
      const rclcpp::Node::SharedPtr& n,
      const std::string& serial,
      const k4a_device_configuration_t& config)  :
      K4AWrapper(serial, config, CONFIG_registered),
      node_(n) {
    boot_timestamp_ = node_->get_clock()->now();
    costmap_publisher_ = 
        node_->create_publisher<sensor_msgs::msg::Image>(CONFIG_costmap_topic, 1);
    cloud_publisher_ = 
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(CONFIG_points_topic, 1);
    scan_publisher_ = 
        node_->create_publisher<sensor_msgs::msg::LaserScan>(CONFIG_scan_topic, 1);
    imu_publisher_ =
        node_->create_publisher<sensor_msgs::msg::Imu>(CONFIG_imu_topic, 1);
    rgb_publisher_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
        CONFIG_rgb_topic, 1);
    depth_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(
        CONFIG_depth_topic, 1);
    rgb_camera_info_publisher_ =
        node_->create_publisher<sensor_msgs::msg::CameraInfo>(
            CONFIG_rgb_camera_info_topic, 1);
    depth_camera_info_publisher_ =
        node_->create_publisher<sensor_msgs::msg::CameraInfo>(
            CONFIG_depth_camera_info_topic, 1);
    InitMessages();
    InitLookups();
  }

  void InitCameraIntrinsics(
      sensor_msgs::msg::CameraInfo* msg,
      const k4a_calibration_camera_t& camera_calibration,
      const std::string& frame_id) {
    const auto& intrinsics = camera_calibration.intrinsics.parameters.param;
    msg->header.frame_id = frame_id;
    msg->width = camera_calibration.resolution_width;
    msg->height = camera_calibration.resolution_height;
    msg->distortion_model = "rational_polynomial";
    msg->d = {intrinsics.k1, intrinsics.k2, intrinsics.p1, intrinsics.p2,
              intrinsics.k3, intrinsics.k4, intrinsics.k5, intrinsics.k6};
    msg->k = {intrinsics.fx, 0.0, intrinsics.cx,
              0.0, intrinsics.fy, intrinsics.cy,
              0.0, 0.0, 1.0};
    msg->r = {1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0};
    msg->p = {intrinsics.fx, 0.0, intrinsics.cx, 0.0,
              0.0, intrinsics.fy, intrinsics.cy, 0.0,
              0.0, 0.0, 1.0, 0.0};
  }

  void InitMessages() {
    rgb_msg_.header.frame_id = CONFIG_rgb_frame;
    depth_msg_.header.frame_id = CONFIG_depth_frame;
    scan_msg_.header.frame_id = CONFIG_scan_frame;
    cloud_msg_.header.frame_id = CONFIG_scan_frame;
    imu_msg_.header.frame_id = CONFIG_imu_frame;

    heightmap_msg_.header = scan_msg_.header;
    // OpenCV Image format, float, 2 channel.
    heightmap_msg_.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
    heightmap_msg_.is_bigendian = false;

    rgb_msg_.format = "jpeg";
    rgb_width_ = calibration_.color_camera_calibration.resolution_width;
    rgb_height_ = calibration_.color_camera_calibration.resolution_height;

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
    InitCameraIntrinsics(
        &rgb_camera_info_msg_,
        calibration_.color_camera_calibration,
        CONFIG_rgb_frame);
    InitCameraIntrinsics(
        &depth_camera_info_msg_,
        calibration_.depth_camera_calibration,
        CONFIG_depth_frame);

    cloud_msg_.fields.resize(4);
    cloud_msg_.point_step = 3 * sizeof(float) + sizeof(uint32_t);
    cloud_msg_.is_dense = false;
    cloud_msg_.is_bigendian = false;
    cloud_msg_.fields[0].name = "x";
    cloud_msg_.fields[0].offset = 0;
    cloud_msg_.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg_.fields[0].count = 1;
    cloud_msg_.fields[1].name = "y";
    cloud_msg_.fields[1].offset = 4;
    cloud_msg_.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg_.fields[1].count = 1;
    cloud_msg_.fields[2].name = "z";
    cloud_msg_.fields[2].offset = 8;
    cloud_msg_.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg_.fields[2].count = 1;
    cloud_msg_.fields[3].name = "rgb";
    cloud_msg_.fields[3].offset = 12;
    cloud_msg_.fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
    cloud_msg_.fields[3].count = 1;
    cloud_msg_.data.resize(width * height * cloud_msg_.point_step);
    cloud_msg_.width = width;
    cloud_msg_.height = height;

    // Signify that the orientation field should be ignored.
    imu_msg_.orientation_covariance.fill(-1);
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

  void PublishPointCloud(const rclcpp::Time& stamp) {
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
    cloud_msg_.header.stamp = ToBuiltinTime(stamp);
    cloud_publisher_->publish(cloud_msg_);
  }

  void DepthToPointCloud(k4a_image_t color_image, k4a_image_t depth_image) {
    static CumulativeFunctionTimer ft(__FUNCTION__);
    CumulativeFunctionTimer::Invocation invoke(&ft);
    uint32_t* rgb_data = nullptr;
    if (FLAGS_points && CONFIG_registered && color_image != nullptr) {
        rgb_data = 
            reinterpret_cast<uint32_t*>(k4a_image_get_buffer(color_image));
    }
    uint16_t* depth_data = 
        reinterpret_cast<uint16_t*>(k4a_image_get_buffer(depth_image));
    // Scan generation only uses every (1 + skip_points) point.
    const size_t incr = FLAGS_points ? 1 : static_cast<size_t>(1 + CONFIG_skip_points);
    for (size_t i = 0; i < points_.size(); i += incr) {
      points_[i] = ext_translation_ + 
          (static_cast<float>(depth_data[i]) * rgbd_ray_lookup_[i]);
      if (rgb_data) {
        colors_[i] = rgb_data[i];
      } else if (FLAGS_points) {
        colors_[i] = 0xC0C0C0LU;
      }
    }
  }

  void PublishScan(const rclcpp::Time& stamp) {
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
    scan_msg_.header.stamp = ToBuiltinTime(stamp);
    scan_publisher_->publish(scan_msg_);
  }

  void PublishRGBImage(k4a_image_t color_image, const rclcpp::Time& stamp) {
    static CumulativeFunctionTimer ft(__FUNCTION__);
    CumulativeFunctionTimer::Invocation invoke(&ft);
    uint32_t* rgb_data = 
          reinterpret_cast<uint32_t*>(k4a_image_get_buffer(color_image));
    const cv::Mat bgra(rgb_height_, rgb_width_, CV_8UC4, rgb_data);
    cv::Mat bgr;
    cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
    cv::imencode(
        ".jpg", bgr, rgb_msg_.data,
        {cv::IMWRITE_JPEG_QUALITY, 90});
    rgb_msg_.header.stamp = ToBuiltinTime(stamp);
    rgb_publisher_->publish(rgb_msg_);
  }

  void PublishDepthImage(k4a_image_t depth_image, const rclcpp::Time& stamp) {
    static CumulativeFunctionTimer ft(__FUNCTION__);
    CumulativeFunctionTimer::Invocation invoke(&ft);
    uint16_t* depth_data =
        reinterpret_cast<uint16_t*>(k4a_image_get_buffer(depth_image));
    memcpy(depth_msg_.data.data(), depth_data, depth_msg_.data.size());
    depth_msg_.header.stamp = ToBuiltinTime(stamp);
    depth_publisher_->publish(depth_msg_);
  }

  void PublishRGBCameraInfo(const rclcpp::Time& stamp) {
    rgb_camera_info_msg_.header.stamp = ToBuiltinTime(stamp);
    rgb_camera_info_publisher_->publish(rgb_camera_info_msg_);
  }

  void PublishDepthCameraInfo(const rclcpp::Time& stamp) {
    depth_camera_info_msg_.header.stamp = ToBuiltinTime(stamp);
    depth_camera_info_publisher_->publish(depth_camera_info_msg_);
  }

  void PublishHeightMap() {
  }

  void ImuCallback(k4a_imu_sample_t& imu_sample) override {
    if (!FLAGS_imu) {
      return;
    }

    // Use the message's timestamp-since-boot because we might be processing
    // from a queue.
    uint64_t sec =
        static_cast<uint64_t>(boot_timestamp_.seconds()) +
        imu_sample.acc_timestamp_usec / 1'000'000;
    uint64_t nsec = static_cast<uint64_t>(boot_timestamp_.nanoseconds() % 1000000000LL) +
                    (imu_sample.acc_timestamp_usec % 1'000'000) * 1'000;

    sec += nsec / 1'000'000'000;
    nsec %= 1'000'000'000;

    imu_msg_.header.stamp.sec = static_cast<int32_t>(sec);
    imu_msg_.header.stamp.nanosec = static_cast<uint32_t>(nsec);
    imu_msg_.angular_velocity.x = imu_sample.gyro_sample.xyz.x;
    imu_msg_.angular_velocity.y = imu_sample.gyro_sample.xyz.y;
    imu_msg_.angular_velocity.z = imu_sample.gyro_sample.xyz.z;
    imu_msg_.linear_acceleration.x = imu_sample.acc_sample.xyz.x;
    imu_msg_.linear_acceleration.y = imu_sample.acc_sample.xyz.y;
    imu_msg_.linear_acceleration.z = imu_sample.acc_sample.xyz.z;

    imu_publisher_->publish(imu_msg_);
  }

  void RGBDCallback(k4a_image_t color_image, k4a_image_t depth_image) {
    rclcpp::Time stamp_time = node_->get_clock()->now();
    if (color_image != nullptr && FLAGS_rgb) {
      PublishRGBImage(color_image, stamp_time);
      PublishRGBCameraInfo(stamp_time);
    } else {
      if (color_image == nullptr) {
        RCLCPP_WARN(node_->get_logger(), "Color image is null");
      }
    }

    if (depth_image == nullptr) return;

    if (FLAGS_depth || FLAGS_points) {
      DepthToPointCloud(color_image, depth_image);
    }

    if (FLAGS_scan) {
      PublishScan(stamp_time);
    }
    if (FLAGS_depth) {
      PublishDepthImage(depth_image, stamp_time);
      PublishDepthCameraInfo(stamp_time);
    }
    if (FLAGS_points) {
      PublishPointCloud(stamp_time);
    }
  }

  void ColorCallback(k4a_image_t image) override {
    rclcpp::Time stamp_time = node_->get_clock()->now();
    if (image != nullptr && FLAGS_rgb) {
      PublishRGBImage(image, stamp_time);
      PublishRGBCameraInfo(stamp_time);
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
  sensor_msgs::msg::LaserScan scan_msg_;
  sensor_msgs::msg::CompressedImage rgb_msg_;
  sensor_msgs::msg::CameraInfo rgb_camera_info_msg_;
  sensor_msgs::msg::CameraInfo depth_camera_info_msg_;
  sensor_msgs::msg::Image depth_msg_;
  sensor_msgs::msg::Image heightmap_msg_;
  sensor_msgs::msg::Imu imu_msg_;
  sensor_msgs::msg::PointCloud2 cloud_msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr costmap_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      rgb_camera_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      depth_camera_info_publisher_;
  rclcpp::Node::SharedPtr node_;
  int rgb_width_ = 0;
  int rgb_height_ = 0;
  // Translation component of extrinsics.
  Eigen::Vector3f ext_translation_;
  rclcpp::Time boot_timestamp_;
};

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("k4a_ros");
  std::string config_file = FLAGS_config_file;
  if (!config_file.empty() && config_file[0] != '/') {
    try {
      config_file =
          ament_index_cpp::get_package_share_directory("k4a_ros") + "/" + config_file;
    } catch (const std::exception& e) {
      RCLCPP_WARN(
          node->get_logger(),
          "Failed to resolve package share config path for '%s': %s",
          FLAGS_config_file.c_str(),
          e.what());
    }
  }
  config_reader::ConfigReader reader({config_file});
  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

  switch (FLAGS_resolution) {
    case 720:
      config.color_resolution = K4A_COLOR_RESOLUTION_720P;
      break;
    case 1080:
      config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
      break;
    case 1440:
      config.color_resolution = K4A_COLOR_RESOLUTION_1440P;
      break;
    case 1536:
      config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
      break;
    case 2160:
      config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
      break;
    case 3072:
      config.color_resolution = K4A_COLOR_RESOLUTION_3072P;
      break;
    default:
      LOG(WARNING) << "Unknown resolution \"" << FLAGS_resolution << "\", defaulting to 720p";
      config.color_resolution = K4A_COLOR_RESOLUTION_720P;
  }

  switch (FLAGS_fps) {
    case 5:
      config.camera_fps = K4A_FRAMES_PER_SECOND_5;
      break;
    case 15:
      config.camera_fps = K4A_FRAMES_PER_SECOND_15;
      break;
    case 30:
      config.camera_fps = K4A_FRAMES_PER_SECOND_30;
      break;
    default:
      LOG(WARNING) << "Unknown fps \"" << FLAGS_fps << "\", defaulting to 15";
      config.camera_fps = K4A_FRAMES_PER_SECOND_15;
  }

  config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  // Depth frames are always required for lidar/scan conversion.
  config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
  config.synchronized_images_only = false;
  DepthToLidar interface(node, CONFIG_serial, config);

  while (rclcpp::ok()) {
    interface.Capture();
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}
