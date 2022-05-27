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
#include <cstdlib>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// Raspberry Pi Camera FOV
const int fov = 62;

tf2_ros::Buffer tfBuffer;

// Window name
const std::string window = "Preview";

// Keep track of laser scan
// std::vector<float> laser_scan;
sensor_msgs::LaserScan current_scan;

// Contour list, used for previews
std::vector<std::vector<cv::Point>> shapes;

// Keep track of the exact points
std::vector<geometry_msgs::PointStamped> fires;
std::vector<geometry_msgs::PointStamped> persons;

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

// Helper function to check if a point is already within another points' 3D
// radius
bool in_radius(const geometry_msgs::PointStamped point,
               const std::vector<geometry_msgs::PointStamped>& points,
               const double R = 1) {
  // grab coords
  double x = point.point.x;
  double y = point.point.y;
  double z = point.point.z;

  // measure distance to every other point in list
  for (const auto& p : points) {
    double dx = std::pow(x - p.point.x, 2);
    double dy = std::pow(y - p.point.y, 2);
    double dz = std::pow(z - p.point.z, 2);

    double distance = std::sqrt(dx + dy + dz);

    // check if the distance is in the radius
    if (distance <= R) {
      return true;
    }
  }

  // It appears the point is not in the list
  return false;
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

// Clean up detected contours, filter by number of edges and compute their
// center points
void filter_copy_point(const std::vector<std::vector<cv::Point>>& contours,
                       const int& sides,
                       std::vector<cv::Point>& points) {
  for (const auto& shape : contours) {
    const double epsilon = 0.1 * cv::arcLength(shape, true);
    std::vector<cv::Point> approx;
    cv::approxPolyDP(shape, approx, epsilon, true);

    // make sure the shape is of the specified type
    if (approx.size() == sides) {
      const cv::Moments m = cv::moments(approx);
      const cv::Point center = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
      points.push_back(center);
    }
  }
}

// Publishers for Blue Square, Red Triangle and Vis Marker
ros::Publisher bs;
ros::Publisher rt;
ros::Publisher vm;

void find_persons(const cv::Mat& hsv,
                  const std_msgs::Header& imghdr,
                  const sensor_msgs::LaserScan::ConstPtr& scn) {
  static int id = 0;

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

  // Determine position of candidate shapes
  std::vector<cv::Point> points;
  filter_copy_point(contours, 4, points);

  for (const auto& point : points) {
    // Calculate deviation from center (-50 to 50%)
    float relative = (point.x - hsv.cols / 2) / (float)(hsv.cols);

    // First check: skip if sign is in the fringes of view (these tend to be
    // innacurate)
    if (relative > 0.333 || relative < -0.333) {
      continue;
    }

    /*
      Use our knowledge of the FOV to determine approximate angle of sign
    */

    // relative angle from camera center at 0°
    float rel_angle = relative * fov;

    // absolute angle from camera center at 0°
    int angle = (360 + (int)rel_angle) % 360;

    // convert relative angle to radians (TODO: use rel or abs angle??)
    float rad = rel_angle * (M_PI / 180.0);

    float dist = 0;
    try {
      dist = (*scn).ranges.at(angle);
    } catch (const std::out_of_range& e) {
      ROS_WARN("Laser Scan index %i was out of range", angle);
      continue;
    }

    // Second check: skip if the sign is too far away
    if (dist > 1) {
      continue;
    }

    // Calculate position of sign
    try {
      geometry_msgs::TransformStamped loc = tfBuffer.lookupTransform(
          "map", imghdr.frame_id, imghdr.stamp, ros::Duration(10));

      geometry_msgs::PointStamped point;

      point.header.frame_id = "map";
      point.header.stamp = ros::Time::now();

      // To calculate the absolute position of the sign, we need the relative
      // rotation of the robot
      double yaw = tf2::getYaw(loc.transform.rotation);

      double theta = yaw - rad;

      point.point.x = loc.transform.translation.x + (dist * std::cos(theta));
      point.point.y = loc.transform.translation.y + (dist * std::sin(theta));

      if (in_radius(point, persons)) {
        continue;
      }

      // add our marker
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "vision";
      marker.id = ++id;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = point.point.x;
      marker.pose.position.y = point.point.y;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      vm.publish(marker);

      bs.publish(point);
      persons.push_back(point);

    } catch (const tf2::ExtrapolationException& e) {
      ROS_ERROR("Error: %s", e.what());
    }
  }
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

  // TODO: same as for persons

  geometry_msgs::PointStamped point;
  rt.publish(point);
}

// Just a dumb old function to check the camera and draw a preview
void callback(const sensor_msgs::Image::ConstPtr& img,
              const sensor_msgs::LaserScan::ConstPtr& scn) {
  cv_bridge::CvImagePtr cv_img_ptr;
  try {
    cv_img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat hsv;
  cv::cvtColor(cv_img_ptr->image, hsv, cv::COLOR_BGR2HSV);

  find_persons(hsv, img->header, scn);
  // find_fires(hsv, img->header);

  // draw_preview(cv_img_ptr->image);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "vision_node");

  cv::namedWindow(window);

  ros::NodeHandle nh;

  tf2_ros::TransformListener tfListener(tfBuffer);

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/image",
                                                            1);
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 1);

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::LaserScan>
      ap;

  message_filters::Synchronizer<ap> sync(ap(10), image_sub, scan_sub);

  sync.registerCallback(boost::bind(&callback, _1, _2));

  bs = nh.advertise<geometry_msgs::PointStamped>("/blue_square_pos", 1);
  rt = nh.advertise<geometry_msgs::PointStamped>("/red_triangle_pos", 1);

  vm = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  ROS_INFO("Vision node has finished initializing!");

  ros::spin();
}