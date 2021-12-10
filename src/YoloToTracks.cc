#include <cfloat>
#include <vector>

#include "amrl_msgs/HumanStateArrayMsg.h"
#include "amrl_msgs/HumanStateMsg.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include <pcl_ros/point_cloud.h>

using amrl_msgs::HumanStateArrayMsg;
using amrl_msgs::HumanStateMsg;
using darknet_ros_msgs::BoundingBoxes;
using darknet_ros_msgs::BoundingBox;
using Eigen::Vector2f;
using Eigen::Vector3f;
using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud;
using ros::Publisher;
using ros::Time;
using std::vector;
using std::cout;
using std::endl;

struct Track;

Publisher human_pub_;
Publisher cloud_pub_;

// Parameters
const static int kMinTrackLength = 2;
const static int kMaxTrackLength = 10;
const static int kMaxAge = 5;
const float kDetectionThreshold = 0.4;
const float kDistThresh = 0.5;
// Color Inrinsics
const static float cx = 642.437622;
const static float cy = 363.448151;
const static float fx = 612.269653;
const static float fy = 612.225464;

HumanStateArrayMsg human_states_;
vector<Track> tracks_;
int id_counter_ = 0;
BoundingBoxes detections_;
vector<Vector3f> current_cloud_;
vector<Vector2f> image_points_;
bool new_detection_ = false;
bool new_lidar_ = false;

struct Track {
    vector<Vector2f> poses_;
    vector<double> times_;
    int id_;
    int age_;
};

void DetectionCb(const BoundingBoxes& msg) {
    detections_ = msg;
    new_detection_ = true;
}

void LidarCb(const PointCloud2& msg) {
    // Rotation
    Eigen::Matrix3f R;
    R << 0.97, 0.0, -.24,
         0.0, 1.0, 0.0,
         .24, 0.0, .97;

    // Translation
    Vector3f translation(-0.05, 0, -.210);

    // Intrinsics
    Eigen::Matrix3f intrinsics;
    intrinsics << fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1;

    sensor_msgs::PointCloud cloud;
    // Convert to cloud
    sensor_msgs::convertPointCloud2ToPointCloud(msg, cloud);

    vector<Vector3f> new_cloud;
    vector<Vector2f> image_coords;
    for (const auto point : cloud.points) {
        if (fabs(point.x) < 10 && fabs(point.y) < 10 and fabs(point.x) > 0) {
            const Vector3f c_point(point.x, point.y, point.z);
            new_cloud.push_back(c_point);
            Vector3f X(-point.y, -point.z, point.x);
            Vector3f Y = X + translation;
            Y = intrinsics * Y;
            Vector2f coord(Y[0] / Y[2], Y[1] / Y[2]);
            image_coords.push_back(coord);
        }
    }
    current_cloud_ = new_cloud;
    image_points_ = image_coords;
    new_lidar_ = true;
}

Vector2f GetVelocity(const Track& track) {
    Vector2f velocity(0, 0);
    if (track.poses_.size() < 2) {
        return velocity;
    }
    // Calculate Velocity from observations/times
    for (size_t i = 1; i < track.poses_.size(); ++i) {
        const double t2 = track.times_[i];
        const double t1 = track.times_[i - 1];
        const double dt = t2 - t1;
        const Vector2f p2 = track.poses_[i];
        const Vector2f p1 = track.poses_[i - 1];
        const Vector2f vel = (p2 - p1) / dt;
        velocity += vel;
    }
    return velocity / (track.poses_.size() - 1);
}

// Estimates the Velocities and Creates the HumanStateArrayMsg
void EstimateVelocity() {
    human_states_.human_states.clear();
    int count = 0;
    // For track in track
    for (const Track& track : tracks_) {
        count++;
        if (track.times_.size() < kMinTrackLength) {
            continue;
        }
        HumanStateMsg human;
        human.pose.x = track.poses_.back().x();
        human.pose.y = track.poses_.back().y();
        human.id = track.id_;
        const Vector2f velocity = GetVelocity(track);
        human.translational_velocity.x = velocity.x();
        human.translational_velocity.y = velocity.y();
        if (track.poses_.back().norm() > 0.01) {
            human_states_.human_states.push_back(human);
        }
    }
}

Vector3f GetFromLidar(const int u,
                      const int v,
                      const int& size_x,
                      const int& size_y) {
    vector<Vector3f> observed_points;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    for (size_t i = 0; i < current_cloud_.size(); ++i) {
        if (i < current_cloud_.size() && i < image_points_.size()) {
            const Vector2f image = image_points_[i];
            const Vector3f cloud = current_cloud_[i];
            const float x = image.x();
            const float y = image.y();
            if (x > u - size_x && x < u + size_x &&
                    y > v - size_y && y < v + size_y) {
                const float wx = cloud.x();
                const float wy = cloud.y();
                const float wz = cloud.z();
                // Only care about things within a certain range
                // need to check the units on these.
                if (wx > 0 && wx < 5 and fabs(wy) < 5) {
                    observed_points.push_back(cloud);
                    pcl::PointXYZ pcl_point;
                    pcl_point.x = wx;
                    pcl_point.y = wy;
                    pcl_point.z = wz;
                    pcl_cloud.push_back(pcl_point);
                }
            }
        }
    }

    // Calculate Average of all points discovered in detection
    // TODO(jaholtz) determine if this is actually reasonable
    Vector3f average_point(0,0);
    for (const Vector3f& point : observed_points) {
        average_point += point;
    }
    if (observed_points.size() > 0) {
        average_point = average_point / observed_points.size();
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(pcl_cloud, output);
    output.header.frame_id = "velodyne";

    // Publish point cloud of detections
    cloud_pub_.publish(output);

    return average_point;
}

vector<Vector3f> GetObservation() {
    vector<Vector3f> poses;
    for (const BoundingBox& box : detections_.bounding_boxes) {
        if (box.Class == "person" && box.probability > kDetectionThreshold) {
            const int u = box.xmin + ((box.xmax - box.xmin) / 2);
            const int v = box.ymin + ((box.ymax - box.ymin) / 2);
            Vector2f pose_estimate(0, 0);
            const Vector3f pose = GetFromLidar(u, v, (box.xmax - box.xmin) / 2,
                                                     (box.ymax - box.ymin) / 2);
            poses.push_back(pose);
        }
    }
    return poses;
}

int GetClosest(const Vector3f& pose,
                float& dist,
               const double& now) {
    if (tracks_.size() < 1) return -1;

    float best_dist = FLT_MAX;
    int best_index = 0;
    int current_index = 0;
    for (Track track : tracks_) {
        const Vector2f pose2(pose.x(), pose.y());
        const float dist = (track.poses_.back() - pose2).norm();
        if (dist < best_dist) {
            best_dist = dist;
            best_index = current_index;
        }
        current_index++;
    }
    dist = best_dist;
    return best_index;
}

void CreateTrack(const Vector3f& pose, const double& now) {
    Track track;
    const Vector2f pose2d(pose.x(), pose.y());
    track.id_ = id_counter_;
    id_counter_++;
    track.age_ = 0;
    track.poses_ = {pose2d};
    track.times_ = {now};
    tracks_.push_back(track);
}

void UpdatePose(const Vector3f& pose, const int index) {
    Vector2f last_pose = tracks_[index].poses_.back();
    const Vector2f pose2d(pose.x(), pose.y());
    last_pose += pose2d;
    last_pose = last_pose / 2;
    tracks_[index].poses_.back() = last_pose;
}

void UpdateTrack(const Vector3f& pose, const double& now, int index) {
    tracks_[index].age_ = 0;
    // if (now - tracks_[index].times_.back() < 0.000001) {
        // UpdatePose(pose, index);
    // } else {
        const Vector2f pose2d(pose.x(), pose.y());
        if (tracks_[index].poses_.size() >= kMaxTrackLength) {
            tracks_[index].poses_.erase(tracks_[index].poses_.begin());
            tracks_[index].times_.erase(tracks_[index].times_.begin());
        }
        tracks_[index].poses_.push_back(pose2d);
        tracks_[index].times_.push_back(now);
    // }
}

void PruneTracks() {
    vector<Track> new_tracks;
    for (auto track : tracks_) {
        track.age_ += 1;
        if (track.age_ <= kMaxAge) {
            new_tracks.push_back(track);
        }
    }
    tracks_ = new_tracks;
}

// Updates all of the tracks as necessary
void UpdateTracks() {
    new_lidar_ = false;
    new_detection_ = false;

    const vector<Vector3f> poses = GetObservation();
    const double now = ros::Time::now().toSec();

    // Using the Observations either associate to existing one or create new track
    for (const Vector3f& pose : poses) {
        float dist = FLT_MAX;
        const int index = GetClosest(pose, dist, now);
        if (dist > kDistThresh) {
            CreateTrack(pose, now);
        } else {
            UpdateTrack(pose, now, index);
        }
    }

    EstimateVelocity();
    PruneTracks();
}

int main(int argc, char** argv) {
  // Initialize ROS.
  ros::init(argc, argv, "YoloTracker");
  ros::NodeHandle n;

  // Publishers
  human_pub_ = n.advertise<HumanStateArrayMsg>("human_states", 1);
  cloud_pub_ = n.advertise<PointCloud2>("tracking_cloud", 1);

  // Subscribers
  ros::Subscriber darknet_sub =
      n.subscribe("/darknet_ros/bounding_boxes", 1, &DetectionCb);
  ros::Subscriber velodyne_sub =
      n.subscribe("/velodyne_points", 1, &LidarCb);

  ros::Rate loop(60.0);
  while (ros::ok()) {
    ros::spinOnce();
    // Update Tracks
    if (new_detection_) {
        UpdateTracks();
    }

    // Publish humans out
    human_pub_.publish(human_states_);

    loop.sleep();
  }
  return 0;
}

