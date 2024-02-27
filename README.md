# k4a_ros

Azure Kinect ROS drivers

[![Build Status](https://github.com/ut-amrl/k4a_ros/actions/workflows/buildTest.yml/badge.svg)](https://github.com/ut-amrl/k4a_ros/actions)

## Dependencies

1. Lua5.1, glog, gflags, tcmalloc, CImg
    ```
    sudo apt install liblua5.1-0-dev libgflags-dev libgoogle-glog-dev libgoogle-perftools-dev cimg-dev
    ```
2. [ROS](https://wiki.ros.org/Installation/), [AMRL ROS Messages](https://github.com/ut-amrl/amrl_msgs)
3. [Microsoft Kinect For Azure SDK](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download)  
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
4. Add the file [99-k4a.rules](99-k4a.rules) to `/etc/udev/rules.d` to allow access to the kinect device, and reboot to apply the changes.
5. Be sure to pull in changes for the git submodules before making. If it's your first time pulling changes in this repo run `git submodule update --init --recursive` first. After doing this, run `git submodule update --recursive --remote`.
6. (Optional) To use microphone array, install the [audio_common](https://wiki.ros.org/audio_common/Tutorials/Streaming%20audio) ros package according to the tutorial. At this point, you can go to audio system settings and verify the Azure Kinect microphone is connected.

## Compile
1. Add it to your `ROS_PACKAGE_PATH` environment variable:
    ```
    export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
    ```

2. Run `make [-j]`

## Usage
The streamed topics will be available on the Fixed Frame `kinect` on rviz by default.

To stream just converted laserscan data :
```
./bin/depth_to_lidar --points=false
```

To stream converted laserscan data and a 3D point cloud:
```
./bin/depth_to_lidar --points=true
```

To stream an RGB point cloud as `sensor_msgs/PointCloud2`:
```
./bin/stream_pcl
```

To save registered color and RGB images to disk (e.g. to the directory `out`):
```
./bin/save_rgbd_images --save_dir out
```

## libusb errors
If you encounter an issue with the libusb driver, it may be due to the data bus limit on the usb port not being set to a high enough value. If this is the case, open the grub file (/etc/default/grub) and replace the line `GRUB_CMDLINE_LINUX_DEFAULT=quiet splash` with `GRUB_CMDLINE_LINUX_DEFAULT=quiet splash usbcore.usbfs_memory_mb=2000`. Then, run `sudo update-grub` and reboot the system after updating grub.

## XServer errors

This package needs access to an OpenGL display for the k4a drivers. If running in a headless mode, you might have to recreate the `~/.Xauthority` file to gain access to the server, in addition to setting the `DISPLAY` environment variable (e.g. `export DISPLAY=:0`). To re-create the `~/.Xauthority` file, see:
https://unix.stackexchange.com/questions/209746/how-to-resolve-no-protocol-specified-for-su-user
