/* Polygon Explorer ROS class
 */

#include "PolygonExplorerNode.h"
#include <glog/logging.h>
#include <functional>

PolygonExplorerNode::PolygonExplorerNode()
    : Node("polygon_explorer_node"),
      odometrySubscription_(
          message_filters::Subscriber<nav_msgs::msg::Odometry>(this,
                                                               "odom_11_odom")),
      laserSubscription_(
          message_filters::Subscriber<sensor_msgs::msg::LaserScan>(
              this, "scan_11_laser_front")),
      timeSynchronizer_(odometrySubscription_, laserSubscription_, 1) {
  polygonExplorer_.setCallBack(this);
  timeSynchronizer_.registerCallback(
      std::bind(&PolygonExplorerNode::subscriberCallback, this,
                std::placeholders::_1, std::placeholders::_2));
} /* -----  end of method PolygonExplorerNode::PolygonExplorerNode
     (constructor)  ----- */

void PolygonExplorerNode::subscriberCallback(
    const std::shared_ptr<nav_msgs::msg::Odometry>& odometry,
    const std::shared_ptr<sensor_msgs::msg::LaserScan>& laser_scan) {
  std::vector<PolygonPoint> polygon_points;
  for (unsigned int i = 0; i < laser_scan->ranges.size(); ++i) {
    double theta = laser_scan->angle_min + i * laser_scan->angle_increment;
    double x = laser_scan->ranges.at(i) * cos(theta);
    double y = laser_scan->ranges.at(i) * sin(theta);

    PointType point_type;
    if (laser_scan->ranges.at(i) >= laser_scan->range_max) {
      point_type = PointType::MAX_RANGE;
    } else {
      point_type = PointType::OBSTACLE;
    }

    polygon_points.emplace_back(x, y, point_type);
  }

  Polygon polygon(polygon_points);

  Pose current_pose;
  convertFromRosGeometryMsg(odometry->pose, current_pose);
  auto position_diff = current_pose.getPosition() - previousPose_.getPosition();
  // Orientation difference of quaternions
  // (http://www.boris-belousov.net/2016/12/01/quat-dist/)
  auto orientation_diff =
      current_pose.getRotation() * previousPose_.getRotation().conjugated();
  // Transform transformation into frame of previous pose
  auto transformation_previous_pose_current_pose =
      Pose(previousPose_.transform(position_diff),
           previousPose_.getRotation() * orientation_diff);

  polygonExplorer_.addPose(transformation_previous_pose_current_pose, polygon);

  previousPose_ = current_pose;
}

void PolygonExplorerNode::convertFromRosGeometryMsg(
    const geometry_msgs::msg::PoseWithCovariance& geometryPoseMsg, Pose& pose) {
  convertFromRosGeometryMsg(geometryPoseMsg.pose.position, pose.getPosition());

  // This is the definition of ROS Geometry pose.
  typedef kindr::RotationQuaternion<double> RotationQuaternionGeometryPoseLike;

  RotationQuaternionGeometryPoseLike rotation;
  convertFromRosGeometryMsg(geometryPoseMsg.pose.orientation, rotation);
  pose.getRotation() = rotation;
}

void PolygonExplorerNode::convertFromRosGeometryMsg(
    const geometry_msgs::msg::Quaternion& geometryQuaternionMsg,
    Rotation& rotationQuaternion) {
  rotationQuaternion.setValues(geometryQuaternionMsg.w, geometryQuaternionMsg.x,
                               geometryQuaternionMsg.y,
                               geometryQuaternionMsg.z);
}

void PolygonExplorerNode::convertFromRosGeometryMsg(
    const geometry_msgs::msg::Point& geometryPointMsg, Position& position) {
  position.x() = geometryPointMsg.x;
  position.y() = geometryPointMsg.y;
  position.z() = geometryPointMsg.z;
}

void PolygonExplorerNode::updateVisualizationCallback(
    const PoseGraph pose_graph) {
  }

  poseGraphVisualizationPublisher_->publish(pose_graph_marker);
}