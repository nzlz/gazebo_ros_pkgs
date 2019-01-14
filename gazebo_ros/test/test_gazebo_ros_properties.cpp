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

// NEW CODE


  //TODO
  void GetModelProperties();

  //TODO
  void GetJointProperties();

  //TODO
  void GetLinkProperties();

  //TODO
  void GetLightProperties();

  //TODO
  void SetJointProperties();

  //TODO
  void SetLinkProperties();

  //TODO
  void SetLightProperties();


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

/*void GazeboRosPropertiesTest::GetModelProperties(
    const std::string & _name)
{

  // Spawn a entity using the gazebo SpawnEntity service.
  auto spawn_request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();

  geometry_msgs::Pose model_pose_;
  model_pose_->position->x = 0.0;
  model_pose_->position->y = 0.0;
  model_pose_->position->z = 0.0;
  model_pose_->orientation->x = 0.0;
  model_pose_->orientation->y= 0.0;
  model_pose_->orientation->z = 0.0;
  model_pose_->orientation->w = 0.0;

  // simple_arm model from public gazebo/models
  spawn_request->model_name = "simple_arm";
  spawn_request->xml ="<model name='simple_arm'> <link name='arm_base'> <inertial> <pose frame=''>0 0 0.00099 0 -0 0</pose> <inertia> <ixx>1.11</ixx> <ixy>0</ixy> <ixz>0</ixz> <iyy>100.11</iyy> <iyz>0</iyz> <izz>1.01</izz> </inertia> <mass>101</mass> </inertial> <collision name='arm_base_geom'> <pose frame=''>0 0 0.05 0 -0 0</pose> <geometry> <box> <size>1 1 0.1</size> </box> </geometry> <max_contacts>10</max_contacts> <surface> <contact> <ode/> </contact> <bounce/> <friction> <torsional> <ode/> </torsional> <ode/> </friction> </surface> </collision> <visual name='arm_base_geom_visual'> <pose frame=''>0 0 0.05 0 -0 0</pose> <geometry> <box> <size>1 1 0.1</size> </box> </geometry> <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/Blue</name> </script> </material> </visual> <collision name='arm_base_geom_arm_trunk'> <pose frame=''>0 0 0.6 0 -0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>1</length> </cylinder> </geometry> <max_contacts>10</max_contacts> <surface> <contact> <ode/> </contact> <bounce/> <friction> <torsional> <ode/> </torsional> <ode/> </friction> </surface> </collision> <visual name='arm_base_geom_arm_trunk_visual'> <pose frame=''>0 0 0.6 0 -0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>1</length> </cylinder> </geometry> <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/Red</name> </script> </material> </visual> <self_collide>0</self_collide> <enable_wind>0</enable_wind> <kinematic>0</kinematic> </link> <link name='arm_shoulder_pan'> <pose frame=''>0 0 1.1 0 -0 0</pose> <inertial> <pose frame=''>0.045455 0 0 0 -0 0</pose> <inertia> <ixx>0.011</ixx> <ixy>0</ixy> <ixz>0</ixz> <iyy>0.0225</iyy> <iyz>0</iyz> <izz>0.0135</izz> </inertia> <mass>1.1</mass> </inertial> <collision name='arm_shoulder_pan_geom'> <pose frame=''>0 0 0.05 0 -0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>0.1</length> </cylinder> </geometry> <max_contacts>10</max_contacts> <surface> <contact> <ode/> </contact> <bounce/> <friction> <torsional> <ode/> </torsional> <ode/> </friction> </surface> </collision> <visual name='arm_shoulder_pan_geom_visual'> <pose frame=''>0 0 0.05 0 -0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>0.1</length> </cylinder> </geometry> <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/Yellow</name> </script> </material> </visual> <collision name='arm_shoulder_pan_geom_arm_shoulder'> <pose frame=''>0.55 0 0.05 0 -0 0</pose> <geometry> <box> <size>1 0.05 0.1</size> </box> </geometry> <max_contacts>10</max_contacts> <surface> <contact> <ode/> </contact> <bounce/> <friction> <torsional> <ode/> </torsional> <ode/> </friction> </surface> </collision> <visual name='arm_shoulder_pan_geom_arm_shoulder_visual'> <pose frame=''>0.55 0 0.05 0 -0 0</pose> <geometry> <box> <size>1 0.05 0.1</size> </box> </geometry> <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/Yellow</name> </script> </material> </visual> <self_collide>0</self_collide> <enable_wind>0</enable_wind> <kinematic>0</kinematic> </link> <link name='arm_elbow_pan'> <pose frame=''>1.05 0 1.1 0 -0 0</pose> <inertial> <pose frame=''>0.0875 0 0.083333 0 -0 0</pose> <inertia> <ixx>0.031</ixx> <ixy>0</ixy> <ixz>0.005</ixz> <iyy>0.07275</iyy> <iyz>0</iyz> <izz>0.04475</izz> </inertia> <mass>1.2</mass> </inertial> <collision name='arm_elbow_pan_geom'> <pose frame=''>0 0 0.1 0 -0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>0.2</length> </cylinder> </geometry> <max_contacts>10</max_contacts> <surface> <contact> <ode/> </contact> <bounce/> <friction> <torsional> <ode/> </torsional> <ode/> </friction> </surface> </collision> <visual name='arm_elbow_pan_geom_visual'> <pose frame=''>0 0 0.1 0 -0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>0.2</length> </cylinder> </geometry> <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/Red</name> </script> </material> </visual> <collision name='arm_elbow_pan_geom_arm_elbow'> <pose frame=''>0.3 0 0.15 0 -0 0</pose> <geometry> <box> <size>0.5 0.03 0.1</size> </box> </geometry> <max_contacts>10</max_contacts> <surface> <contact> <ode/> </contact> <bounce/> <friction> <torsional> <ode/> </torsional> <ode/> </friction> </surface> </collision> <visual name='arm_elbow_pan_geom_arm_elbow_visual'> <pose frame=''>0.3 0 0.15 0 -0 0</pose> <geometry> <box> <size>0.5 0.03 0.1</size> </box> </geometry> <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/Yellow</name> </script> </material> </visual> <collision name='arm_elbow_pan_geom_arm_wrist'> <pose frame=''>0.55 0 0.15 0 -0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>0.3</length> </cylinder> </geometry> <max_contacts>10</max_contacts> <surface> <contact> <ode/> </contact> <bounce/> <friction> <torsional> <ode/> </torsional> <ode/> </friction> </surface> </collision> <visual name='arm_elbow_pan_geom_arm_wrist_visual'> <pose frame=''>0.55 0 0.15 0 -0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>0.3</length> </cylinder> </geometry> <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/Red</name> </script> </material> </visual> <self_collide>0</self_collide> <enable_wind>0</enable_wind> <kinematic>0</kinematic> </link> <link name='arm_wrist_lift'> <pose frame=''>1.6 0 1.05 0 -0 0</pose> <inertial> <pose frame=''>0 0 0 0 -0 0</pose> <inertia> <ixx>0.01</ixx> <ixy>0</ixy> <ixz>0</ixz> <iyy>0.01</iyy> <iyz>0</iyz> <izz>0.001</izz> </inertia> <mass>0.1</mass> </inertial> <collision name='arm_wrist_lift_geom'> <pose frame=''>0 0 0.5 0 -0 0</pose> <geometry> <cylinder> <radius>0.03</radius> <length>1</length> </cylinder> </geometry> <max_contacts>10</max_contacts> <surface> <contact> <ode/> </contact> <bounce/> <friction> <torsional> <ode/> </torsional> <ode/> </friction> </surface> </collision> <visual name='arm_wrist_lift_geom_visual'> <pose frame=''>0 0 0.5 0 -0 0</pose> <geometry> <cylinder> <radius>0.03</radius> <length>1</length> </cylinder> </geometry> <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/Yellow</name> </script> </material> </visual> <self_collide>0</self_collide> <enable_wind>0</enable_wind> <kinematic>0</kinematic> </link> <link name='arm_wrist_roll'> <pose frame=''>1.6 0 1 0 -0 0</pose> <inertial> <pose frame=''>0 0 0 0 -0 0</pose> <inertia> <ixx>0.01</ixx> <ixy>0</ixy> <ixz>0</ixz> <iyy>0.01</iyy> <iyz>0</iyz> <izz>0.001</izz> </inertia> <mass>0.1</mass> </inertial> <collision name='arm_wrist_roll_geom'> <pose frame=''>0 0 0.025 0 -0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>0.05</length> </cylinder> </geometry> <max_contacts>10</max_contacts> <surface> <contact> <ode/> </contact> <bounce/> <friction> <torsional> <ode/> </torsional> <ode/> </friction> </surface> </collision> <visual name='arm_wrist_roll_geom_visual'> <pose frame=''>0 0 0.025 0 -0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>0.05</length> </cylinder> </geometry> <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/Red</name> </script> </material> </visual> <self_collide>0</self_collide> <enable_wind>0</enable_wind> <kinematic>0</kinematic> </link> <joint name='arm_shoulder_pan_joint' type='revolute'> <parent>arm_base</parent> <child>arm_shoulder_pan</child> <axis> <dynamics> <damping>1</damping> <friction>0</friction> <spring_reference>0</spring_reference> <spring_stiffness>0</spring_stiffness> </dynamics> <xyz>0 0 1</xyz> <use_parent_model_frame>1</use_parent_model_frame> <limit> <lower>-1e+16</lower> <upper>1e+16</upper> </limit> </axis> </joint> <joint name='arm_elbow_pan_joint' type='revolute'> <parent>arm_shoulder_pan</parent> <child>arm_elbow_pan</child> <axis> <dynamics> <damping>1</damping> <friction>0</friction> <spring_reference>0</spring_reference> <spring_stiffness>0</spring_stiffness> </dynamics> <xyz>0 0 1</xyz> <use_parent_model_frame>1</use_parent_model_frame> <limit> <lower>-1e+16</lower> <upper>1e+16</upper> </limit> </axis> </joint> <joint name='arm_wrist_lift_joint' type='prismatic'> <parent>arm_elbow_pan</parent> <child>arm_wrist_lift</child> <axis> <dynamics> <damping>1</damping> <friction>0</friction> <spring_reference>0</spring_reference> <spring_stiffness>0</spring_stiffness> </dynamics> <limit> <lower>-0.8</lower> <upper>0.1</upper> </limit> <xyz>0 0 1</xyz> <use_parent_model_frame>1</use_parent_model_frame> </axis> </joint> <joint name='arm_wrist_roll_joint' type='revolute'> <parent>arm_wrist_lift</parent> <child>arm_wrist_roll</child> <axis> <dynamics> <damping>1</damping> <friction>0</friction> <spring_reference>0</spring_reference> <spring_stiffness>0</spring_stiffness> </dynamics> <limit> <lower>-2.99999</lower> <upper>2.99999</upper> </limit> <xyz>0 0 1</xyz> <use_parent_model_frame>1</use_parent_model_frame> </axis> </joint> <pose frame=''>0.256397 -2.58283 0 0 -0 0</pose> </model>";
  spawn_request->robot_namespace = "";
  spawn_request->initial_pose = model_pose_;
  spawn_request->reference_frame = "";

  auto spawn_entity_client = node->create_client<std_srvs::srv::Empty>("spawn_entity");
  ASSERT_NE(nullptr, spawn_entity_client);
  EXPECT_TRUE(spawn_entity_client->wait_for_service(std::chrono::seconds(1)));

  auto response_future = spawn_entity_client->async_send_request(spawn_request);
  EXPECT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node_, response_future));

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

  
  EXPECT_EQ(response->parent_model_name, )
  EXPECT_EQ(response->canonical_body_name, )
  EXPECT_EQ(response->body_names, )
  EXPECT_EQ(response->geom_names, )
  EXPECT_EQ(response->joint_names, )
  EXPECT_EQ(response->child_model_names, )
  EXPECT_EQ(response->is_static, )

}*/


// Example ros2 service call
//ros2 service call /demo/get_joint_properties 'gazebo_msgs/GetJointProperties' "{joint_name: 'simple_arm::arm_shoulder_pan_joint'}"
//gazebo_msgs.srv.GetJointProperties_Response(type=0, damping=[], position=[0.27383861369841345], rate=[-3.5745634939922096e-05], success=True, status_message='GetJointProperties: got properties')
/*void GazeboRosPropertiesTest::GetJointProperties(
  const std::string & _name,
  const std::vector<std::float64 1> & damping[],
  const std::vector<std::float64 1> & position[],
  const std::vector<std::float64 1> & rate[],
  )

{
}*/

/*void GazeboRosPropertiesTest::SetJointProperties(

)
{
// Response
string joint_name                               # name of joint
gazebo_msgs/ODEJointProperties ode_joint_config

ode_joint_config
float64[] damping             # joint damping
float64[] hiStop              # joint limit
float64[] loStop              # joint limit
float64[] erp                 # set joint erp
float64[] cfm                 # set joint cfm
float64[] stop_erp            # set joint erp for joint limit "contact" joint
float64[] stop_cfm            # set joint cfm for joint limit "contact" joint
float64[] fudge_factor        # joint fudge_factor applied at limits, see ODE manual for info.
float64[] fmax                # ode joint param fmax
float64[] vel                 # ode joint param vel

}*/

void GazeboRosPropertiesTest::GetLinkProperties(
  const std::string & _name,
  const ignition::math::Pose3d & _pose,
  const std::bool & _gravity_mode,
  const std:float64 & _mass,
  const std:float64 & _ixx,
  const std:float64 & _ixy,
  const std:float64 & _ixz,
  const std:float64 & _iyy,
  const std:float64 & _iyz,
  const std:float64 & _izz)
{  
  auto entity = world_->EntityByName(_name);
  ASSERT_NE(nullptr, entity);

  auto request = std::make_shared<gazebo_msgs::srv::GetLinkProperties::Request>();
  request->link_name = _name;

  auto response_future = get_link_properties_client_->async_send_request(request);
  EXPECT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node_, response_future));

  auto response = response_future.get();
  ASSERT_NE(nullptr, response);
  EXPECT_TRUE(response->success);

  EXPECT_NEAR(_pose.Pos().X(), response->com.position.x, tol) << _name;
  EXPECT_NEAR(_pose.Pos().Y(), response->com.position.y, tol) << _name;
  EXPECT_NEAR(_pose.Pos().Z(), response->com.position.z, tol) << _name;

  EXPECT_NEAR(_pose.Rot().X(), response->com.orientation.x, tol) << _name;
  EXPECT_NEAR(_pose.Rot().Y(), response->com.orientation.y, tol) << _name;
  EXPECT_NEAR(_pose.Rot().Z(), response->com.orientation.z, tol) << _name;
  EXPECT_NEAR(_pose.Rot().W(), response->com.orientation.w, tol) << _name;

  EXPECT_EQ(_gravity_mode, response->gravity_mode) << _name;
  EXPECT_EQ(_mass, response->_mass) << _name;
  EXPECT_EQ(_ixx, response->_ixx) << _name;
  EXPECT_EQ(_ixy, response->_ixy) << _name;
  EXPECT_EQ(_ixz, response->_ixz) << _name;
  EXPECT_EQ(_iyy, response->_iyy) << _name;
  EXPECT_EQ(_iyz, response->_iyz) << _name;
  EXPECT_EQ(_izz, response->_izz) << _name;
}

void GazeboRosPropertiesTest::SetLinkProperties(
  const std::string & _name,
  const ignition::math::Pose3d & _pose,
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
  request->link_name = _name;
  request->com.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(_pose.Pos());
  request->com.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(_pose.Rot());
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


/*void GazeboRosPropertiesTest::GetLightProperties(
  const std::string & _name,
  const std::array<std::float64 > & param[]
  )

{
}
//ros2 service call /demo/get_light_properties 'gazebo_msgs/GetLightProperties' "{light_name: 'sun'}"
//gazebo_msgs.srv.GetLightProperties_Response(diffuse=std_msgs.msg.ColorRGBA(r=0.800000011920929, g=0.800000011920929, b=0.800000011920929, a=1.0), attenuation_constant=0.8999999761581421, attenuation_linear=0.009999999776482582, attenuation_quadratic=0.0010000000474974513, success=True, status_message='')


/*
EXAMPLE -STATE

void GazeboRosStateTest::GetState(
  const std::string & _entity,
  const ignition::math::Pose3d & _pose,
  const ignition::math::Vector3d & _lin_vel,
  const ignition::math::Vector3d & _ang_vel)
{
  auto entity = world_->EntityByName(_entity);
  ASSERT_NE(nullptr, entity);

  auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
  request->name = _entity;

  auto response_future = get_state_client_->async_send_request(request);
  EXPECT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node_, response_future));

  auto response = response_future.get();
  ASSERT_NE(nullptr, response);
  EXPECT_TRUE(response->success);

  EXPECT_NEAR(_pose.Pos().X(), response->state.pose.position.x, tol) << _entity;
  EXPECT_NEAR(_pose.Pos().Y(), response->state.pose.position.y, tol) << _entity;
  EXPECT_NEAR(_pose.Pos().Z(), response->state.pose.position.z, tol) << _entity;

  EXPECT_NEAR(_pose.Rot().X(), response->state.pose.orientation.x, tol) << _entity;
  EXPECT_NEAR(_pose.Rot().Y(), response->state.pose.orientation.y, tol) << _entity;
  EXPECT_NEAR(_pose.Rot().Z(), response->state.pose.orientation.z, tol) << _entity;
  EXPECT_NEAR(_pose.Rot().W(), response->state.pose.orientation.w, tol) << _entity;

  EXPECT_NEAR(_lin_vel.X(), response->state.twist.linear.x, tol) << _entity;
  EXPECT_NEAR(_lin_vel.Y(), response->state.twist.linear.y, tol) << _entity;
  EXPECT_NEAR(_lin_vel.Z(), response->state.twist.linear.z, tol) << _entity;

  EXPECT_NEAR(_ang_vel.X(), response->state.twist.angular.x, tol) << _entity;
  EXPECT_NEAR(_ang_vel.Y(), response->state.twist.angular.y, tol) << _entity;
  EXPECT_NEAR(_ang_vel.Z(), response->state.twist.angular.z, tol) << _entity;
}

void GazeboRosStateTest::SetState(
  const std::string & _entity,
  const ignition::math::Pose3d & _pose,
  const ignition::math::Vector3d & _lin_vel,
  const ignition::math::Vector3d & _ang_vel)
{
  auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
  request->state.name = _entity;
  request->state.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(_pose.Pos());
  request->state.pose.orientation =
    gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(_pose.Rot());
  request->state.twist.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(_lin_vel);
  request->state.twist.angular = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(_ang_vel);

  auto response_future = set_state_client_->async_send_request(request);
  EXPECT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node_, response_future));

  auto response = response_future.get();
  ASSERT_NE(nullptr, response);
  EXPECT_TRUE(response->success);
}

TEST_F(GazeboRosStateTest, GetSet)
{
  // Get / set model state
  {
    // Get initial state
    this->GetState("boxes", ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));

    // Set new state
    this->SetState("boxes", ignition::math::Pose3d(1.0, 2.0, 10.0, 0, 0, 0),
      ignition::math::Vector3d(4.0, 0, 0), ignition::math::Vector3d::Zero);

    // Check new state
    this->GetState("boxes", ignition::math::Pose3d(1.0, 2.0, 10.0, 0, 0, 0),
      ignition::math::Vector3d(4.0, 0, 0), ignition::math::Vector3d::Zero);
  }

  // Get / set light state
  {
    // Get initial state
    this->GetState("sun", ignition::math::Pose3d(0, 0, 10, 0, 0, 0));

    // Set new state
    this->SetState("sun", ignition::math::Pose3d(1.0, 2.0, 3.0, 0.1, 0.2, 0.3));

    // Check new state
    this->GetState("sun", ignition::math::Pose3d(1.0, 2.0, 3.0, 0.1, 0.2, 0.3));
  }

  // Get / set link state
  {
    // Get initial state - note that is was moved with the model
    this->GetState("boxes::top", ignition::math::Pose3d(1.0, 2.0, 11.25, 0, 0, 0),
      ignition::math::Vector3d(4.0, 0, 0), ignition::math::Vector3d::Zero);

    // Set new state
    this->SetState("boxes::top", ignition::math::Pose3d(10, 20, 30, 0.1, 0, 0),
      ignition::math::Vector3d(1.0, 2.0, 3.0), ignition::math::Vector3d(0.0, 0.0, 4.0));

    // Check new state
    this->GetState("boxes::top", ignition::math::Pose3d(10, 20, 30, 0.1, 0, 0),
      ignition::math::Vector3d(1.0, 2.0, 3.0), ignition::math::Vector3d(0.0, 0.0, 4.0));
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}






