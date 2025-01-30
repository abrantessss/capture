/*
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Luís Abrantes
 */

#include <gazebo_claw_plugin.hpp>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <capture_msgs/msg/claw.hpp>

#include <memory>
#include <mutex>

namespace gazebo
{
  class GazeboClawPluginPrivate
  {
  public:
    gazebo::event::ConnectionPtr update_connection_;
    gazebo::physics::ModelPtr model_;
    gazebo::physics::JointPtr claw_arm_joint_;
    gazebo::physics::JointPtr claw_left_finger_joint_;
    gazebo::physics::JointPtr claw_right_finger_joint_;

    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<capture_msgs::msg::Claw>::SharedPtr position_sub_;

    double claw_arm_position_ = 0.0;
    double claw_fingers_position = 0.0;

    std::mutex lock_;

    void OnUpdate();
    void OnRosMessage(const capture_msgs::msg::Claw::SharedPtr msg);
  };

  GazeboClawPlugin::GazeboClawPlugin()
  : impl_(std::make_unique<GazeboClawPluginPrivate>()) {
    std::cout << "Gazebo Claw Plugin Initialized" << std::endl;
  }

  GazeboClawPlugin::~GazeboClawPlugin() {
    std::cout << "Gazebo Claw Plugin Destroyed" << std::endl;
  }

  void GazeboClawPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    impl_->model_ = model;

    impl_->claw_arm_joint_ = model->GetJoint("claw_joint");
    if (!impl_->claw_arm_joint_) {
      std::cout << "Claw joint not found, unable to start plugin" << std::endl;
      return;
    }

    impl_->claw_left_finger_joint_ = model->GetJoint("left_finger_joint");
    if (!impl_->claw_left_finger_joint_) {
      std::cout << "Claw joint not found, unable to start plugin" << std::endl;
      return;
    }

    impl_->claw_right_finger_joint_ = model->GetJoint("right_finger_joint");
    if (!impl_->claw_right_finger_joint_) {
      std::cout << "Claw joint not found, unable to start plugin" << std::endl;
      return;
    }

    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

    impl_->position_sub_ = impl_->ros_node_->create_subscription<capture_msgs::msg::Claw>(
        "capture/claw_angle", 10,
        std::bind(&GazeboClawPluginPrivate::OnRosMessage, impl_.get(), std::placeholders::_1));

    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboClawPluginPrivate::OnUpdate, impl_.get()));

    std::cout << "Plugin 'gazebo_claw_plugin' carregado para o modelo: " << impl_->model_->GetName() << std::endl;
  }

  void GazeboClawPluginPrivate::OnRosMessage(const capture_msgs::msg::Claw::SharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(lock_);
    claw_arm_position_ = msg->arm;
    claw_fingers_position = msg->fingers;
    std::cout << "Received arm position: " << claw_arm_position_ << " | fingers position: " << claw_fingers_position << std::endl;
  }

  void GazeboClawPluginPrivate::OnUpdate()
  {
      std::lock_guard<std::mutex> guard(lock_);
      double current_arm_pos = claw_arm_joint_->Position(0);
      
      if(claw_arm_position_ >= 0) {
        if (current_arm_pos < claw_arm_position_- 0.2)
          claw_arm_joint_->SetForce(0, 0.12);
        else
          claw_arm_joint_->SetForce(0, 1);
      } else {
        if (current_arm_pos > claw_arm_position_+ 0.2)
          claw_arm_joint_->SetForce(0, 0.08);
        else
          claw_arm_joint_->SetForce(0, -1);
      }
      if(claw_fingers_position <= 0) {
        claw_left_finger_joint_->SetForce(0, 0.001);
        claw_right_finger_joint_->SetForce(0, -0.001);
      } else {
        claw_left_finger_joint_->SetForce(0, -2);
        claw_right_finger_joint_->SetForce(0, 2);
      }
      
      

  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboClawPlugin)
}
