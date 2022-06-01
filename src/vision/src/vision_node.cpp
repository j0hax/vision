#include "vision_node.h"

tf2_ros::Buffer tfBuffer;

// Keep track of the exact points
std::vector<geometry_msgs::PointStamped> fires;
std::vector<geometry_msgs::PointStamped> persons;

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

      // Skip if the area is too small
      if (m.m00 < MIN_AREA) {
        continue;
      }

      const cv::Point center = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
      points.push_back(center);
    }
  }
}

std::vector<geometry_msgs::PointStamped> localize_publish_point(
    const cv::Mat& hsv,
    const std_msgs::Header& imghdr,
    const sensor_msgs::LaserScan::ConstPtr& scn,
    const std::vector<cv::Point>& points) {
  std::vector<geometry_msgs::PointStamped> output;

  for (const auto& point : points) {
    // Calculate deviation from center (-50 to 50%)
    double relative = (point.x - hsv.cols / 2) / (double)(hsv.cols);

    // First check: skip if sign is outside of the middle limit
    if (std::abs(relative) > LIM) {
      continue;
    }

    /*
      Use our knowledge of the FOV to determine approximate angle of sign
    */

    // relative angle from camera center at 0°
    double rel_angle = relative * FOV;

    // absolute angle from camera center at 0°
    int angle = (360 + (int)rel_angle) % 360;

    // convert relative angle to radians (TODO: use rel or abs angle??)
    double rad = rel_angle * (M_PI / 180.0);

    double dist = 0;
    try {
      dist = (*scn).ranges.at(angle);
    } catch (const std::out_of_range& e) {
      ROS_WARN("Laser Scan index %i was out of range", angle);
      continue;
    }

    // Second check: skip if the sign is too far away
    if (dist > MAX_DIST) {
      continue;
    }

    // Calculate position of sign
    try {
      geometry_msgs::TransformStamped loc = tfBuffer.lookupTransform(
          "map", imghdr.frame_id, imghdr.stamp, ros::Duration(1));

      geometry_msgs::PointStamped ps;

      ps.header.frame_id = "map";
      ps.header.stamp = ros::Time::now();

      // To calculate the absolute position of the sign, we need the relative
      // rotation of the robot
      double yaw = tf2::getYaw(loc.transform.rotation);

      double theta = yaw - rad;

      ps.point.x = loc.transform.translation.x + (dist * std::cos(theta));
      ps.point.y = loc.transform.translation.y + (dist * std::sin(theta));

      output.push_back(ps);

    } catch (const tf2::ExtrapolationException& e) {
      ROS_ERROR("Error: %s", e.what());
    }
  }
  return output;
}

// Publishers for Blue Square, Red Triangle and Vis Marker
ros::Publisher bs;
ros::Publisher rt;
ros::Publisher vm;

// Marker ID counter for RViz
std::size_t marker_id = 0;

void find_persons(const cv::Mat& hsv,
                  const std_msgs::Header& imghdr,
                  const sensor_msgs::LaserScan::ConstPtr& scn) {
  // filter blue-ish colors
  cv::Mat filtered;
  cv::Scalar min_b = cv::Scalar(210 / 2, 255 / 2, 0);
  cv::Scalar max_b = cv::Scalar(255 / 2, 255, 255);
  cv::inRange(hsv, min_b, max_b, filtered);

  // Track our Contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(filtered, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  // Determine position of candidate shapes
  std::vector<cv::Point> points;
  filter_copy_point(contours, 4, points);

  // localize each of these 2D Points
  std::vector<geometry_msgs::PointStamped> ps =
      localize_publish_point(hsv, imghdr, scn, points);

  // Publish the points and add them to the list of known points
  for (const auto& p : ps) {
    if (in_radius(p, persons)) {
      continue;
    }

    bs.publish(p);

    persons.push_back(p);

    // add our marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "vision";
    marker.id = ++marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = p.point.x;
    marker.pose.position.y = p.point.y;
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
  }
}

void find_fires(const cv::Mat& hsv,
                const std_msgs::Header& imghdr,
                const sensor_msgs::LaserScan::ConstPtr& scn) {
  // filter red-ish colors
  cv::Mat filtered;
  cv::Scalar min_b = cv::Scalar(0 / 2, 255 / 2, 0);
  cv::Scalar max_b = cv::Scalar(15 / 2, 255, 255);
  cv::inRange(hsv, min_b, max_b, filtered);

  // Track our Contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(filtered, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  // Determine position of candidate shapes
  std::vector<cv::Point> points;
  filter_copy_point(contours, 3, points);

  // localize each of these 2D Points
  std::vector<geometry_msgs::PointStamped> ps =
      localize_publish_point(hsv, imghdr, scn, points);

  // Publish the points and add them to the list of known points
  for (const auto& p : ps) {
    if (in_radius(p, fires)) {
      continue;
    }

    rt.publish(p);

    fires.push_back(p);

    // add our marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "vision";
    marker.id = ++marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = p.point.x;
    marker.pose.position.y = p.point.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    vm.publish(marker);
  }
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
  find_fires(hsv, img->header, scn);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "vision_node");

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