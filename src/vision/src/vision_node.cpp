#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"

tf2_ros::Buffer tfBuffer;

// Window name
const std::string window = "Preview";

// Keep track of seen squares and triangles
std::vector<std::vector<cv::Point>> shapes;

// Draws an OpenCV Preview using a preview image and the list of objects
void draw_preview(const cv::Mat& preview) {
  // draw an outline
  cv::drawContours(preview, shapes, 0, cv::Scalar(0, 255, 0));

  // mark each of our shapes
  for (const auto& shape : shapes) {
    cv::Moments m = cv::moments(shape);

    // mark center
    cv::Point center = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
    cv::drawMarker(preview, center, cv::Scalar(0, 255, 0));

    // show points
    std::string text = std::to_string(shape.size()) + " points";
    cv::Point tp = cv::Point(center.x, center.y - 50);
    cv::putText(preview, text, tp, cv::FONT_HERSHEY_PLAIN, 1,
                cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
  }
  shapes.clear();
  cv::imshow(window, preview);
  cv::waitKey(1);
}

// Helper function to clean up contours and copy them to the shapes array
void filter_copy_shape(const std::vector<std::vector<cv::Point>>& contours,
                       const int& sides,
                       std::vector<std::vector<cv::Point>>& shapes) {
  for (const auto& shape : contours) {
    double epsilon = 0.1 * cv::arcLength(shape, true);
    std::vector<cv::Point> approx;
    cv::approxPolyDP(shape, approx, epsilon, true);

    // make sure the shape is of the specified type
    if (approx.size() == sides) {
      shapes.push_back(approx);
    }
  }
}

// Publishers for Blue Square and Red Triangle
ros::Publisher bs;
ros::Publisher rt;

void find_persons(cv::Mat& hsv) {
  // filter blue-ish colors
  cv::Mat filtered;
  cv::Scalar min_b = cv::Scalar(210 / 2, 255 / 2, 0);
  cv::Scalar max_b = cv::Scalar(255 / 2, 255, 255);
  cv::inRange(hsv, min_b, max_b, filtered);

  // Track our Contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(filtered, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  // Filter out other crap data
  filter_copy_shape(contours, 4, shapes);

  geometry_msgs::PointStamped point;
  rt.publish(point);
}

void find_fires(cv::Mat& hsv) {
  // filter red-ish colors
  cv::Mat filtered;
  cv::Scalar min_b = cv::Scalar(0 / 2, 255 / 2, 0);
  cv::Scalar max_b = cv::Scalar(15 / 2, 255, 255);
  cv::inRange(hsv, min_b, max_b, filtered);

  // Track our Contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(filtered, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  // Filter out other crap data
  filter_copy_shape(contours, 3, shapes);

  geometry_msgs::PointStamped point;
  rt.publish(point);
}

// Just a dumb old function to check the camera and draw a preview
void image_callback(const sensor_msgs::ImageConstPtr& img) {
  cv_bridge::CvImagePtr cv_img_ptr;
  try {
    cv_img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat hsv;
  cv::cvtColor(cv_img_ptr->image, hsv, cv::COLOR_BGR2HSV);

  find_persons(hsv);
  find_fires(hsv);

  // localization test
  if (shapes.size() > 0) {
    try {
      geometry_msgs::TransformStamped loc =
          tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
      ROS_INFO("Have marker at [%f, %f]", loc.transform.translation.x,
               loc.transform.translation.y);
    } catch (const tf2::ExtrapolationException& e) {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  draw_preview(cv_img_ptr->image);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "vision_node");

  cv::namedWindow(window);

  ros::NodeHandle nh;

  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Subscriber s = nh.subscribe("/camera/image", 10, image_callback);

  bs = nh.advertise<geometry_msgs::PointStamped>("/blue_square_pos", 10);
  rt = nh.advertise<geometry_msgs::PointStamped>("/red_triangle_pos", 10);

  ROS_INFO("Vision node has finished initializing!");

  ros::spin();
}