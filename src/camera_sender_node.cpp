#include <ros/ros.h>
#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_sender_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  int device_num, image_width, image_height, loop_rate;

  if (!nh.getParam("/usb_cam/video_device_num", device_num))
  {
    ROS_ERROR("Error retrieving parameter video_device_num");
    device_num = 0;
  }

  if (!nh.getParam("/usb_cam/image_width", image_width))
  {
    ROS_ERROR("Error retrieving parameter image_width");
    image_width = 640;
  }

  if (!nh.getParam("/usb_cam/image_height", image_height))
  {
    ROS_ERROR("Error retrieving parameter image_height");
    image_height = 480;
  }

  if (!nh.getParam("/usb_cam/loop_rate", loop_rate))
  {
    ROS_ERROR("Error retrieving parameter loop_rate");
    loop_rate = 60;
  }

  cv::VideoCapture capture(device_num);

  if (!capture.isOpened())
  {
    ROS_ERROR("Video Device can not be opened");
    return 1;
  }

  int queue_size = image_width*image_height;
  image_transport::Publisher publisher = it.advertise("/camera/image_raw", queue_size);

  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate rate(loop_rate);
  while(nh.ok())
  {
    capture >> frame;

    if (!frame.empty())
    {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      publisher.publish(msg);
      cv::waitKey(1);
    }
    ros::spinOnce();
    rate.sleep();
  }
}
