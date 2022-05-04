#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_listener.h>

ros::Publisher pub;

void callback(const sensor_msgs::ImageConstPtr &img) {
  cv_bridge::CvImagePtr cv_img_ptr;
  try {
    cv_img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat processed_img;
  /* RGB-Bild in HSV-Farbraum umändern:
   * https://docs.opencv.org/4.2.0/d8/d01/group__imgproc__color__conversions.html#gga4e0972be5de079fed4e3a10e24ef5ef0aa4a7f0ecf2e94150699e48c79139ee12
   */
  cv::cvtColor(cv_img_ptr->image, processed_img, cv::COLOR_BGR2HSV);

  pub.publish(cv_bridge::CvImage(cv_img_ptr->header,
                                 sensor_msgs::image_encodings::BGR8,
                                 processed_img)
                  .toImageMsg());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vision_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/camera/image", 10, callback);
  // ROS-Einführung s. 54: "Binärbild für jede Objektfarbe erzeugen"
  pub = nh.advertise<sensor_msgs::Image>("/binary/red", 10);
  // TODO: weitere Farbeinstellungen und Datentypen veröffentlichen
  ros::spin();
}