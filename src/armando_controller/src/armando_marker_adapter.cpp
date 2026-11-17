#include <memory>

#include <cmath>
#include <Eigen/Core>
#include <kdl/frames.hpp>

#include "aruco_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class armando_marker_adapter : public rclcpp::Node
{
public:
  armando_marker_adapter()
  : Node("armando_marker_adapter")
  {
    declare_parameter<int>("target_id", 0);
    declare_parameter<double>("offset_d", 0.05);

    aruco_pose = this->create_subscription<aruco_msgs::msg::MarkerArray>(
      "/aruco_marker_publisher/markers", 10,
      std::bind(&armando_marker_adapter::aruco_pose_callback, this, _1));

    target_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 10);

  }
  
private:
  rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr aruco_pose;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub;

  void aruco_pose_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg)
  {

  int target_id = this->get_parameter("target_id").as_int();
  double d = this->get_parameter("offset_d").as_double();
  double qx, qy, qz, qw;

  // Scorri tutti i marker rilevati in questo frame
  for (const auto & marker : msg->markers) {
    if (static_cast<int>(marker.id) == target_id) {

      // KDL::Frame del marker legando posizione e quaternione estratti da 'marker'
      KDL::Frame T_ref_marker(KDL::Rotation::Quaternion(marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w), KDL::Vector(marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z));
      
      // Traslazione d lungo Z e rotazione di pi attorno a X
      KDL::Frame T_marker_target(KDL::Rotation::RotX(M_PI), KDL::Vector(0.0, 0.0, d));

      KDL::Frame T_ref_target = T_ref_marker * T_marker_target;
      
      // Prepara e pubblica il messaggio PoseStamped
      geometry_msgs::msg::PoseStamped target_msg;
      target_msg.header = msg->header;


      // Riempi target_msg.pose.position e target_msg.pose.orientation
      //       estraendo i valori da T_ref_target
      target_msg.pose.position.x = T_ref_target.p.x();
      target_msg.pose.position.y = T_ref_target.p.y();
      target_msg.pose.position.z = T_ref_target.p.z();

      T_ref_target.M.GetQuaternion(qx, qy, qz, qw);
      target_msg.pose.orientation.x = qx;
      target_msg.pose.orientation.y = qy;
      target_msg.pose.orientation.z = qz;
      target_msg.pose.orientation.w = qw;

      target_pose_pub->publish(target_msg);
      return; 
    }
  }
}
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<armando_marker_adapter>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("armando_marker_adapter"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}