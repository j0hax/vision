#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

/* Constants */

// Raspberry Pi Camera FOV
const std::size_t FOV = 62;

// Inner limit of a sign on a frame: rule of thirds
const double LIM = 1 / 3.0;

// Minimum size for a sign
const std::size_t MIN_AREA = 500;

// Maximum distance a sign should be
const float MAX_DIST = 1;

/* Function definitions */

/*
   Helper function to check if a point is already within another points' 3D
   radius
*/
bool in_radius(const geometry_msgs::PointStamped point,
               const std::vector<geometry_msgs::PointStamped>& points,
               const double R);

/*
   Clean up detected contours, filter by number of edges and compute their
   center points
*/
void filter_copy_point(const std::vector<std::vector<cv::Point>>& contours,
                       const int& sides,
                       std::vector<cv::Point>& points);

/*
   Given Image and Lidar data, calculate point position
*/
std::vector<geometry_msgs::PointStamped> localize_publish_point(
    const cv::Mat& hsv,
    const std_msgs::Header& imghdr,
    const sensor_msgs::LaserScan::ConstPtr& scn,
    const std::vector<cv::Point>& points);

/*
   Check for and localize blue squares
*/
void find_persons(const cv::Mat& hsv,
                  const std_msgs::Header& imghdr,
                  const sensor_msgs::LaserScan::ConstPtr& scn);

/*
   Check for and localize red triangles
*/
void find_fires(const cv::Mat& hsv,
                const std_msgs::Header& imghdr,
                const sensor_msgs::LaserScan::ConstPtr& scn);

/*
   Main callback function to process synchronized image and scan data
*/
void callback(const sensor_msgs::Image::ConstPtr& img,
              const sensor_msgs::LaserScan::ConstPtr& scn);