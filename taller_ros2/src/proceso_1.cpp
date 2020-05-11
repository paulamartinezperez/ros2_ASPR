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
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "practica_msgs/srv/get_string.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class NodePublisher : public rclcpp::Node
{
public:
  NodePublisher(const std::string & name, const std::chrono::nanoseconds & rate)
  : Node(name), counter_(0)
  {
    pub_ = create_publisher<std_msgs::msg::String>(
      "topic_AB", rclcpp::QoS(100).best_effort());

    timer_ = create_wall_timer(
      rate, std::bind(&NodePublisher::timer_callback, this));
  }

  void timer_callback()
  {
    message.data = "Message: " + std::to_string(counter_++);

    RCLCPP_INFO(get_logger(), "Publishing [%s] in %s", message.data.c_str(), get_name());

    pub_->publish(message);
  }

private:
  std_msgs::msg::String message;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

  int counter_;
};


class NodeSubscriber_Client : public rclcpp::Node
{
public:
  NodeSubscriber_Client()
  : Node("nodo_B")
  {
    sub_ = create_subscription<std_msgs::msg::String>(
      "topic_AB", rclcpp::QoS(100).best_effort(),
      std::bind(&NodeSubscriber_Client::callback, this, _1));

    service_ = create_service<practica_msgs::srv::GetString>(
      "getstring", std::bind(&NodeSubscriber_Client::server_string, this, _1, _2));
  }


  void callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: [%s] in %s",
      msg->data.c_str(), get_name());
    resp = msg->data.c_str();
  }

  void server_string(
    const std::shared_ptr<practica_msgs::srv::GetString::Request> request,
    std::shared_ptr<practica_msgs::srv::GetString::Response> response)
  {
    response->num = resp;

    RCLCPP_INFO(get_logger(), "Incoming request [%s]", request->a.c_str());
    RCLCPP_INFO(get_logger(), "sending back response: [%s]",
      response->num.c_str());
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Service<practica_msgs::srv::GetString>::SharedPtr service_;

  std::string resp;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_A = std::make_shared<NodePublisher>("nodo_A", 50ms);
  auto node_B = std::make_shared<NodeSubscriber_Client>();

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node_A);
  executor.add_node(node_B);


  executor.spin();

  rclcpp::shutdown();
  return 0;
}
