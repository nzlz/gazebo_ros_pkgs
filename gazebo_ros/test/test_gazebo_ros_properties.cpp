// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gazebo/test/ServerFixture.hh>

#include <gazebo_msgs/srv/get_world_properties.hpp>
#include <gazebo_msgs/srv/get_model_properties.hpp>
#include <gazebo_msgs/srv/get_joint_properties.hpp>
#include <gazebo_msgs/srv/get_link_properties.hpp>
#include <gazebo_msgs/srv/get_light_properties.hpp>
#include <gazebo_msgs/srv/get_physics_properties.hpp>
#include <gazebo_msgs/srv/set_joint_properties.hpp>
#include <gazebo_msgs/srv/set_link_properties.hpp>
#include <gazebo_msgs/srv/set_light_properties.hpp>
#include <gazebo_msgs/srv/set_physics_properties.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#define tol 10e-2

class GazeboRosPropertiesTest : public gazebo::ServerFixture
{
public:
  // Documentation inherited
  void SetUp() override;

  //TODO
  void GetModelProperties();

  //TODO
  void GetJointProperties();

  void GetLinkProperties(
  const std::string & _link_name,
  const std::bool & _gravity_mode,
  const std:float64 & _mass,
  const std:float64 & _ixx,
  const std:float64 & _ixy,
  const std:float64 & _ixz,
  const std:float64 & _iyy,
  const std:float64 & _iyz,
  const std:float64 & _izz);

  void GetLightProperties(
  const std::string & _light_name,
  const ignition::math::Vector4d & _diffuse,
  const std:float64 & _attenuation_constant,
  const std:float64 & _attenuation_linear,
  const std:float64 & _attenuation_quadratic);

  //TODO
  void SetJointProperties();

  void SetLinkProperties(
  const std::string & _link_name,
  const std::bool & _gravity_mode,
  const std:float64 & _mass,
  const std:float64 & _ixx,
  const std:float64 & _ixy,
  const std:float64 & _ixz,
  const std:float64 & _iyy,
  const std:float64 & _iyz,
  const std:float64 & _izz);

  void SetLightProperties(
  const std::string & _light_name,
  const ignition::math::Vector4d & _diffuse,
  const std:float64 & _attenuation_constant,
  const std:float64 & _attenuation_linear,
  const std:float64 & _attenuation_quadratic);


  gazebo::physics::WorldPtr world_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::GetModelProperties>> get_model_properties_client_;
  std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::GetJointProperties>> get_joint_properties_client_;
  std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::GetLinkProperties>> get_link_properties_client_;
  std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::GetLightProperties>> get_light_properties_client_;
  std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::SetJointProperties>> set_joint_properties_client_;
  std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::SetLinkProperties>> set_link_properties_client_;
  std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::SetLightProperties>> set_light_properties_client_;

};

void GazeboRosPropertiesTest::SetUp()
{
  // Load world with state plugin and start paused
  this->Load("worlds/gazebo_ros_properties_test.world", true);

  // World
  world_ = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world_);

  // Create ROS clients
  node_ = std::make_shared<rclcpp::Node>("gazebo_ros_properties_test");
  ASSERT_NE(nullptr, node_);

  get_model_properties_client_ =
    node_->create_client<gazebo_msgs::srv::GetModelProperties>("test/get_model_properties");
  ASSERT_NE(nullptr, get_model_properties_client_);
  EXPECT_TRUE(get_model_properties_client_->wait_for_service(std::chrono::seconds(1)));

  get_model_properties_client_ =
    node_->create_client<gazebo_msgs::srv::GetJointProperties>("test/get_joint_properties");
  ASSERT_NE(nullptr, get_model_properties_client_);
  EXPECT_TRUE(get_model_properties_client_->wait_for_service(std::chrono::seconds(1)));

  get_link_properties_client_ =
    node_->create_client<gazebo_msgs::srv::GetLinkProperties>("test/get_link_properties");
  ASSERT_NE(nullptr, get_link_properties_client_);
  EXPECT_TRUE(get_link_properties_client_->wait_for_service(std::chrono::seconds(1)));

  get_light_properties_client_ =
    node_->create_client<gazebo_msgs::srv::GetLightProperties>("test/get_light_properties");
  ASSERT_NE(nullptr, get_light_properties_client_);
  EXPECT_TRUE(get_light_properties_client_->wait_for_service(std::chrono::seconds(1)));

  set_joint_properties_client_ =
    node_->create_client<gazebo_msgs::srv::SetJointProperties>("test/set_joint_properties");
  ASSERT_NE(nullptr, set_joint_properties_client_);
  EXPECT_TRUE(set_joint_properties_client_->wait_for_service(std::chrono::seconds(1)));

  set_link_properties_client_ =
    node_->create_client<gazebo_msgs::srv::SetLinkProperties>("test/set_link_properties");
  ASSERT_NE(nullptr, set_link_properties_client_);
  EXPECT_TRUE(set_link_properties_client_->wait_for_service(std::chrono::seconds(1)));

  set_light_properties_client_ =
    node_->create_client<gazebo_msgs::srv::SetLightProperties>("test/set_light_properties");
  ASSERT_NE(nullptr, set_light_properties_client_);
  EXPECT_TRUE(set_light_properties_client_->wait_for_service(std::chrono::seconds(1)));

}

void GazeboRosPropertiesTest::GetModelProperties(
    const std::string & _name)
{

  // Get spawned Model properties
  auto entity = world_->EntityByName(_entity);
  ASSERT_NE(nullptr, entity);

  auto request = std::make_shared<gazebo_msgs::srv::GetModelProperties::Request>();
  request->model_name = _entity;

  auto response_future = get_model_properties_client_->async_send_request(request);
  EXPECT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node_, response_future));

  auto response = response_future.get();
  ASSERT_NE(nullptr, response);
  EXPECT_TRUE(response->success);

  // gazebo models simple_arm
  EXPECT_EQ(response->parent_model_name, '');
  EXPECT_EQ(response->canonical_body_name, '');
  EXPECT_EQ(response->body_names, ['arm_base', 'arm_shoulder_pan', 'arm_elbow_pan', 'arm_wrist_lift', 'arm_wrist_roll']);
  EXPECT_EQ(response->geom_names, ['arm_base_geom', 'arm_base_geom_arm_trunk', 'arm_shoulder_pan_geom', 'arm_shoulder_pan_geom_arm_shoulder', 'arm_elbow_pan_geom', 'arm_elbow_pan_geom_arm_elbow', 'arm_elbow_pan_geom_arm_wrist', 'arm_wrist_lift_geom', 'arm_wrist_roll_geom']);
  EXPECT_EQ(response->joint_names, []);
  EXPECT_EQ(response->child_model_names);
  EXPECT_FALSE(response->is_static);

}

/*void GazeboRosPropertiesTest::GetJointProperties()

{
}
*/

/*
void GazeboRosPropertiesTest::SetJointProperties()
{
}
*/

void GazeboRosPropertiesTest::GetLinkProperties(
  const std::string & _link_name,
  const std::bool & _gravity_mode,
  const std:float64 & _mass,
  const std:float64 & _ixx,
  const std:float64 & _ixy,
  const std:float64 & _ixz,
  const std:float64 & _iyy,
  const std:float64 & _iyz,
  const std:float64 & _izz)
{  
  auto entity = world_->EntityByName(_link_name);
  ASSERT_NE(nullptr, entity);

  auto request = std::make_shared<gazebo_msgs::srv::GetLinkProperties::Request>();
  request->link_name = _link_name;

  auto response_future = get_link_properties_client_->async_send_request(request);
  EXPECT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node_, response_future));

  auto response = response_future.get();
  ASSERT_NE(nullptr, response);
  EXPECT_TRUE(response->success);

  EXPECT_EQ(_gravity_mode, response->gravity_mode) << _link_name;
  EXPECT_EQ(_mass, response->_mass) << _link_name;
  EXPECT_EQ(_ixx, response->_ixx) << _link_name;
  EXPECT_EQ(_ixy, response->_ixy) << _link_name;
  EXPECT_EQ(_ixz, response->_ixz) << _link_name;
  EXPECT_EQ(_iyy, response->_iyy) << _link_name;
  EXPECT_EQ(_iyz, response->_iyz) << _link_name;
  EXPECT_EQ(_izz, response->_izz) << _link_name;
}

void GazeboRosPropertiesTest::SetLinkProperties(
  const std::string & _link_name,
  const std::bool & _gravity_mode,
  const std:float64 & _mass,
  const std:float64 & _ixx,
  const std:float64 & _ixy,
  const std:float64 & _ixz,
  const std:float64 & _iyy,
  const std:float64 & _iyz,
  const std:float64 & _izz)
{

  auto request = std::make_shared<gazebo_msgs::srv::SetLinkProperties::Request>();
  request->link_name = _link_name;
  request->gravity_mode = _gravity_mode;
  request->mass = _mass;
  request->ixx = _ixx;
  request->ixy = _ixy;
  request->ixy = _ixy;
  request->iyy = _iyy;
  request->iyz = _iyz;
  request->izz = _izz;

  auto response_future = set_link_properties_client_->async_send_request(request);
  EXPECT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node_, response_future));

  auto response = response_future.get();
  ASSERT_NE(nullptr, response);
  EXPECT_TRUE(response->success);
}

void GazeboRosPropertiesTest::GetLightProperties(
  const std::string & _light_name,
  const ignition::math::Vector4d & _diffuse,
  const std:float64 & _attenuation_constant,
  const std:float64 & _attenuation_linear,
  const std:float64 & _attenuation_quadratic)
{
  auto entity = world_->EntityByName(_light_name);
  ASSERT_NE(nullptr, entity);

  auto request = std::make_shared<gazebo_msgs::srv::GetLightProperties::Request>();
  request->_light_name = _light_name;

  auto response_future = get_light_properties_client_->async_send_request(request);
  EXPECT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node_, response_future));

  auto response = response_future.get();
  ASSERT_NE(nullptr, response);
  EXPECT_TRUE(response->success);

  EXPECT_NEAR(_diffuse.X(), response->diffuse.r, tol) << _light_name;
  EXPECT_NEAR(_diffuse.Y(), response->diffuse.g, tol) << _light_name;
  EXPECT_NEAR(_diffuse.Z(), response->diffuse.b, tol) << _light_name;
  EXPECT_NEAR(_diffuse.W(), response->diffuse.a, tol) << _light_name;

  EXPECT_NEAR(_attenuation_constant, response->attenuation_constant, tol) << _light_name;
  EXPECT_NEAR(_attenuation_linear, response->attenuation_linear, tol) << _light_name;
  EXPECT_NEAR(_attenuation_quadratic, response->attenuation_quadratic, tol) << _light_name;

}

void GazeboRosPropertiesTest::SetLightProperties(
  const std::string & _light_name,
  const ignition::math::Vector4d & _diffuse,
  const std:float64 & _attenuation_constant,
  const std:float64 & _attenuation_linear,
  const std:float64 & _attenuation_quadratic)
{
  auto request = std::make_shared<gazebo_msgs::srv::SetLightProperties::Request>();
  request->_light_name = _light_name;
  request->diffuse.r = _diffuse.X();
  request->diffuse.g = _diffuse.Y();
  request->diffuse.b = _diffuse.Z();
  request->diffuse.a = _diffuse.W();
  request->attenuation_constant = _attenuation_constant;
  request->attenuation_linear = _attenuation_linear;
  request->attenuation_quadratic = _attenuation_quadratic;

  auto response_future = set_link_properties_client_->async_send_request(request);
  EXPECT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node_, response_future));

  auto response = response_future.get();
  ASSERT_NE(nullptr, response);
  EXPECT_TRUE(response->success);
}

TEST_F(GazeboRosPropertiesTest, GetSet)
{
  // Get / set link properties
  {

    // Get initial link properties
    this->GetLinkProperties("simple_arm::arm_base", 
                          ignition::math::Pose3d(0, 0, 0, 0, 0, 0),
                          true, 101.0, 1.11, 0.0, 0.0, 100.11, 0.0, 1.01);

    // Set link properties
    this->SetLinkProperties("simple_arm::arm_base", 
                          ignition::math::Pose3d(2.0, 2.0, 2.0, 0, 0, 0),
                          true, 102.2, 1.2, 0.2, 0.2, 102.2, 0.2, 1.02);

    // Check new link properties
    this->GetLinkProperties("simple_arm::arm_base", 
                          ignition::math::Pose3d(2.0, 2.0, 2.0, 0, 0, 0),
                          true, 102.2, 1.2, 0.2, 0.2, 102.2, 0.2, 1.02);
  }

  // Get / set light properties
  {

    // Get initial light properties
    this->GetLightProperties("sun", ignition::math::Vector4d(0.800000011920929, 0.800000011920929, 0.800000011920929, 1.0)
                            0.8999999761581421, 0.009999999776482582, 0.0010000000474974513)

    // Set light properties
    this->SetLightProperties("sun", ignition::math::Vector4d(0.92, 0.92, 0.92, 1.02)
                            0.92, 0.0092, 0.002)

    // Check new light properties
    this->GetLightProperties("sun", ignition::math::Vector4d(0.92, 0.92, 0.92, 1.02)
                            0.92, 0.0092, 0.002)
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}






