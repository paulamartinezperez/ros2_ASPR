// Copyright 2020 Paula Martinez
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
#include <stdlib.h>
#include <time.h>

#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "practica_msgs/srv/get_pose_stamped.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using std::placeholders::_2;
unsigned int seed = time(NULL);

class LifeCycleNodePub : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifeCycleNodePub()
  : rclcpp_lifecycle::LifecycleNode("nodo_C")
  {
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/topic_CD", 100);

    timer_ = create_wall_timer(100ms,
        std::bind(&LifeCycleNodePub::timer_callback, this));
  }

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(),
      state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(),
      state.label().c_str());
    pub_->on_activate();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
      state.label().c_str());
    pub_->on_deactivate();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(),
      state.label().c_str());
    pub_.reset();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(),
      state.label().c_str());
    pub_.reset();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_error(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(),
      state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }

  void timer_callback()
  {
    if (pub_->is_activated()) {
      geometry_msgs::msg::PoseStamped pose_msg;

      pose_msg.pose.position.x = rand_r(&seed);
      pose_msg.pose.position.y = rand_r(&seed);
      pose_msg.pose.position.z = rand_r(&seed);

      RCLCPP_INFO(get_logger(), "Publishing [%f, %f, %f] in %s", pose_msg.pose.position.x,
        pose_msg.pose.position.y, pose_msg.pose.position.z, get_name());

      pub_->publish(pose_msg);
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};


class LifeCycleNodeSub_srv : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifeCycleNodeSub_srv()
  : rclcpp_lifecycle::LifecycleNode("nodo_D")
  {
  }

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(),
      state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(),
      state.label().c_str());

    service_ = create_service<practica_msgs::srv::GetPoseStamped>(
      "getposestamped", std::bind(&LifeCycleNodeSub_srv::server_string, this, _1, _2));

    sub2_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "topic_CD", 100, std::bind(&LifeCycleNodeSub_srv::callback2, this, _1));

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
      state.label().c_str());


    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(),
      state.label().c_str());


    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(),
      state.label().c_str());


    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_error(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(),
      state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }

  void callback2(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: [%f, %f, %f] in %s", msg->pose.position.x,
      msg->pose.position.y, msg->pose.position.z, get_name());
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    z = msg->pose.position.z;
  }

  void server_string(
    const std::shared_ptr<practica_msgs::srv::GetPoseStamped::Request> request,
    std::shared_ptr<practica_msgs::srv::GetPoseStamped::Response> response)
  {
    response->x = x;
    response->y = y;
    response->z = z;

    RCLCPP_INFO(get_logger(), "Incoming request [%s]", request->a.c_str());
    RCLCPP_INFO(get_logger(), "sending back response: [%f, %f, %f]",
      response->x, response->y, response->z);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub2_;
  rclcpp::Service<practica_msgs::srv::GetPoseStamped>::SharedPtr service_;
  float x;
  float y;
  float z;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_C = std::make_shared<LifeCycleNodePub>();

  auto node_D = std::make_shared<LifeCycleNodeSub_srv>();

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node_C->get_node_base_interface());
  executor.add_node(node_D->get_node_base_interface());


  executor.spin();


  rclcpp::shutdown();
  return 0;
}
