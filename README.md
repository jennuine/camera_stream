## camera_stream
ROS camera stream package


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
