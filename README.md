# camera_stream
ROS camera stream package

## Launch ROS's camera calibrator

```
roslaunch camera_stream camera_stream_publisher.launch
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 25.4 image:=/usb_cam/image_raw camera:=/usb_cam
```

Refer to [ROS Monocular Camera Calibration Tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) for more information.

### Save and reuse camera calibration parameters

Once camera is calibrated, click save and commit. Wrote calibration data to `/tmp/calibrationdata.tar.gz`.

Copy to some location then `tar -xzf calibrationdata.tar.gz`.

Then create launch file, ex:

```
<launch>
  <arg name="video_device_num" default="0" />
  <arg name="video_device" default="/dev/video$(arg video_device_num)" />
  <arg name="image_width" default="640" />
  <arg name="image_height" default="480" />
  <arg name="loop_rate" default="60" />

  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
    <param name="video_device_num" value="$(arg video_device_num)" />
    <param name="video_device" value="$(arg video_device)" />
    <param name="image_width" value="$(arg image_width)" />
    <param name="image_height" value="$(arg image_height)"/>
    <param name="loop_rate" value="$(arg loop_rate)"/>

    <param name="pixel_format" value="mjpeg" />
    <param name="camera_frame_id" value="usb_cam" />
    <param name="io_method" value="mmap"/>

    <param name="camera_info_url" type="string" value="file://$(find camera_stream)/camera_calibration/usb_cam_calibration.yaml" />
  </node>
</launch>

```

(Found answer [here](https://answers.ros.org/question/231569/reuse-camera-calibration-parameters/))

***

## Remote Streaming

### Host Computer

In the command line:

```
hostname -I
export ROS_HOSTNAME=[output from hostname -I]
export ROS_MASTER_URI=http://[output from hostname -I]:11311
roscore
rosrun camera_stream camera_sender_node
```

or

```
# set environment variable & trim white spaces from hostname -I output
roshost = $(hostname -I | xargs)
export ROS_HOSTNAME=$roshost
export ROS_MASTER_URI=http://$roshost:11311
roscore
rosrun camera_stream camera_sender_node
```

After starting `roscore`, you should see `ROS_MASTER_URI` set to the IP address (the same output from `hostname -I`).

### Client Computer

In the command line:

**Method (1)**

Register all nodes to ROS Master from remote host computer.

```
roshost = [IP from the host computer]
export ROS_IP=$roshost
export ROS_MASTER_URI=http://$roshost:11311
rosrun camera_stream camera_receiver_node
```

**Method (2)**

Register only the camera receiver node to ROS Master from remote host computer.

Edit `ros::init` in `main` from [camera_receiver_node.cpp](https://github.com/jennuine/camera_stream/blob/master/src/camera_receiver_node.cpp) file

```C++
std::map<std::string,std::string> remappings;
remappings["__master"] = ROS_MASTER_URI // what is set to ROS_MASTER_URI on host computer
remappings["__hostname"] = ROS_HOSTNAME // what is set to ROS_HOSTNAME on host computer
ros::init(remappings,"camera_receiver_node");
```
