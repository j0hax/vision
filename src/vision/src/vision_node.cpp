#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"

// Window name
const std::string window = "Preview";

// Publishers for Blue Square and Red Triangle
ros::Publisher bs;
ros::Publisher rt;

void person_callback(const sensor_msgs::ImageConstPtr& img) {
  cv_bridge::CvImagePtr cv_img_ptr;
  try {
    cv_img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  /*
   * RGB-Bild in HSV-Farbraum umändern:
   * https://docs.opencv.org/4.2.0/d8/d01/group__imgproc__color__conversions.html#gga4e0972be5de079fed4e3a10e24ef5ef0aa4a7f0ecf2e94150699e48c79139ee12
   */
  cv::Mat hsv;
  cv::cvtColor(cv_img_ptr->image, hsv, cv::COLOR_BGR2HSV);

  /*
   * HSV-Bild nach Farbschwellenwert filtern
   * Minimum and maximum hues for blue:
   * ca. 100 to 140 degrees, saturation half to full, any brightness
   */

  cv::Mat filtered;
  cv::Scalar min_b = cv::Scalar(100, 255 / 2, 0);
  cv::Scalar max_b = cv::Scalar(140, 255, 255);

  cv::inRange(hsv, min_b, max_b, filtered);

  /*
   * Search for contours in the binary image:
   * RETR_EXTERNAL = retrieves only the extreme outer contours.
   * CHAIN_APPROX_SIMPLE = compress into end points
   * TODO: decide if matchShapes() is a better alternative
   */
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(filtered, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point>> quads;

  // Filter out other crap data
  for (const auto& shape : contours) {
    double epsilon = 0.1 * cv::arcLength(shape, true);
    std::vector<cv::Point> approx;
    cv::approxPolyDP(shape, approx, epsilon, true);

    // make sure the shape is a quadrilateral
    if (approx.size() == 4) {
      quads.push_back(approx);
    }
  }

  // BEGIN OpenCV Preview Code
  cv::Mat preview = cv_img_ptr->image.clone();

  cv::drawContours(preview, quads, 0, cv::Scalar(0, 255, 0));

  for (const auto& shape : quads) {
    // calculate moments to find center
    cv::Moments m = cv::moments(shape);
    cv::Point center = cv::Point(m.m10 / m.m00, m.m01 / m.m00);

    cv::drawMarker(preview, center, cv::Scalar(0, 255, 0), cv::MARKER_STAR);

    std::string text = std::to_string(shape.size()) + " points";
    cv::Point tp = cv::Point(center.x, center.y - 50);
    cv::putText(preview, text, tp, cv::FONT_HERSHEY_PLAIN, 1,
                cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
  }

  cv::imshow(window, preview);
  cv::waitKey(1);
  // END OpenCV Preview Code

  // TODO: determine position via tf2

  geometry_msgs::PointStamped point;

  bs.publish(point);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "vision_node");

  cv::namedWindow(window);

  // TODO: create a seperate nodehandler and callback for red triangle
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/camera/image", 10, person_callback);

  bs = nh.advertise<geometry_msgs::PointStamped>("/blue_square_pos", 10);
  rt = nh.advertise<geometry_msgs::PointStamped>("/red_triangle_pos", 10);

  ros::spin();
}