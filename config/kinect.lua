serial = "";
costmap_topic = "kinect_costmap";
points_topic = "kinect_points";
image_topic = "kinect_image";
image_frame = "kinect";
scan_frame = "base_link";
scan_topic = "kinect_scan";

rotation = {
  yaw = 0;
  pitch = 17;
  roll = 0;
};

translation = {
  x = 0;
  y = 0;
  z = .60;
};

skip_points = 10;
ground_dist_thresh = 0.04;
ground_angle_thresh = 5.0; -- Degrees.
camera_angle_thresh = 55.0;
num_ranges = 180;

