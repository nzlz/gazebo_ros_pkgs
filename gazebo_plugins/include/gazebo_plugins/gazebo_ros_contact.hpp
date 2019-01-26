#ifndef GAZEBO_ROS__GAZEBO_ROS_CONTACT_HPP_
#define GAZEBO_ROS__GAZEBO_ROS_CONTACT_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_ros
{

class GazeboRosContactPrivate;

/// Contactializes ROS with the system arguments passed to Gazebo (i.e. calls rclcpp::init) and
/// provides services to spawn and delete entities from the world.
class GazeboRosContact : public gazebo::SystemPlugin
{
public:
  /// Constructor
  GazeboRosContact();

  /// Destructor
  virtual ~GazeboRosContact();

  // Documentation inherited
  void Load(int argc, char ** argv) override;

private:
  std::unique_ptr<GazeboRosContactPrivate> impl_;
};

}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__GAZEBO_ROS_CONTACT_HPP_