# k4a_ros
Azure Kinect to 2D scan converter

## Dependencies

1. [Microsoft Kinect For Azure SDK](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download)
1. [ROS](https://wiki.ros.org/Installation/)

## Compile

Run `make [-j]`

## Usage

To stream an RGB point cloud as `sensor_msgs/PointCloud2`:
```
./bin/stream_pcl
```

To save registered color and RGB images to disk (e.g. to the directory `out`):
```
./bin/save_rgbd_images --save_dir out
```
