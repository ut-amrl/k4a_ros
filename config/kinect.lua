serial = "";
costmap_topic = "kinect_costmap";
points_topic = "kinect_points";
image_topic = "kinect_image";
image_frame = "kinect";
scan_frame = "base_link";
scan_topic = "scan";

rotation = {
  yaw = 2;
  pitch = 26;
  roll = 2;
};

translation = {
  x = 0;
  y = 0;
  z = 0.45;
};

skip_points = 10;
ground_dist_thresh = 0.02;
ground_angle_thresh = 2.0; -- Degrees.
num_ranges = 180;