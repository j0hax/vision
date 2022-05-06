#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_listener.h>

// Publishers for Blue Square and Red Triangle
ros::Publisher bs;
ros::Publisher rt;

void person_callback(const sensor_msgs::ImageConstPtr &img) {
  cv_bridge::CvImagePtr cv_img_ptr;
  try {
    cv_img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  ROS_INFO("Processing image...");

  /* RGB-Bild in HSV-Farbraum umändern:
   * https://docs.opencv.org/4.2.0/d8/d01/group__imgproc__color__conversions.html#gga4e0972be5de079fed4e3a10e24ef5ef0aa4a7f0ecf2e94150699e48c79139ee12
   */
  cv::Mat hsv;

  cv::cvtColor(cv_img_ptr->image, hsv, cv::COLOR_BGR2HSV);

  /* HSV-Bild nach Farbschwellenwert filtern:
   * https://docs.opencv.org/4.2.0/da/d97/tutorial_threshold_inRange.html
   */

  /*
   * Minimum and maximum hues for blue:
   * ca. 180 to 270 degrees
   * TODO: find appropriate saturation and values for blues, replace `m`
   */
  cv::Mat filtered;
  const double m = 255.0 / 2;

  cv::Scalar min_b = cv::Scalar(180, m, m);
  cv::Scalar max_b = cv::Scalar(270, m, m);

  cv::inRange(hsv, min_b, max_b, filtered);

  /*
   * Search for contours in the binary image:
   * https://docs.opencv.org/4.2.0/d3/dc0/group__imgproc__shape.html#gadf1ad6a0b82947fa1fe3c3d497f260e0
   * RETR_EXTERNAL = retrieves only the extreme outer contours.
   * CHAIN_APPROX_SIMPLE = compress into end points
   * TODO: decide if matchShapes() is a better alternative
   */
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(filtered, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  // TODO: process output array to determine if shape requirements are met

  // TODO: determine position via tf2

  geometry_msgs::PointStamped point;

  bs.publish(point);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vision_node");

  // TODO: create a seperate nodehandler and callback for red triangle
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/camera", 10, person_callback);

  bs = nh.advertise<geometry_msgs::PointStamped>("/blue_square_pos", 10);
  rt = nh.advertise<geometry_msgs::PointStamped>("/red_triangle_pos", 10);

  ros::spin();
}