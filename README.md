# k4a_ros
Azure Kinect ROS drivers

## Dependencies

1. Lua5.1, glog, gflags, tcmalloc, CImg
    ```
    sudo apt install liblua5.1-0-dev libgflags-dev libgoogle-glog-dev libgoogle-perftools-dev cimg-dev
    ```
1. [ROS](https://wiki.ros.org/Installation/)
1. [Microsoft Kinect For Azure SDK](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download)  
    If installing on \*buntu 18.04, you can follow the instructions as is.  
    If installing on \*buntu 20.04, you will need to manually add the repo for 18.04 instead as follows in `/etc/apt/sources.list`:
    ```
    deb https://packages.microsoft.com/ubuntu/18.04/prod bionic main
    ```
    If installing on the nvidia Jetson with Ubuntu 18.04, you will need to modify the package line as follows ([reference](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#debian-package)):
    ```
    deb https://packages.microsoft.com/ubuntu/18.04/multiarch/prod bionic main
    ```
    After that, follow the same instructions as for \*buntu 18.04:
    ```
    sudo apt install libk4a1.4-dev k4a-tools
    ```

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
