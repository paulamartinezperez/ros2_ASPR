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
#include <future>
#include <list>

#include "rclcpp/rclcpp.hpp"
#include "practica_msgs/srv/get_string.hpp"
#include "practica_msgs/srv/get_pose_stamped.hpp"

using namespace std::chrono_literals;

using SharedResponse = practica_msgs::srv::GetString::Response::SharedPtr;
using SharedFuture = std::shared_future<SharedResponse>;
using SharedResponsePose = practica_msgs::srv::GetPoseStamped::Response::SharedPtr;
using SharedFuturePose = std::shared_future<SharedResponsePose>;

class Client : public rclcpp::Node
{
public:
  Client()
  : Node("node_E")
  {
    client_ = create_client<practica_msgs::srv::GetString>("getstring");
    client_stamped_ = create_client<practica_msgs::srv::GetPoseStamped>("getposestamped");

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }

    timer_ = create_wall_timer(1s, std::bind(&Client::timer_callback, this));

    timer_stamped_ = create_wall_timer(1s, std::bind(&Client::timer_stamped_callback, this));
  }

  void timer_callback()
  {
    auto request = std::make_shared<practica_msgs::srv::GetString::Request>();
    request->a = "What is the last number received in node_B?";

    pending_responses_.push_back(client_->async_send_request(request));

    auto rp = pending_responses_.begin();

    while (rp != pending_responses_.end()) {
      if (rp->valid() && rp->wait_for(100ms) == std::future_status::ready) {
        auto resp = rp->get();
        RCLCPP_INFO(get_logger(), "Node B: [%s]",
          resp->num.c_str());

        rp = pending_responses_.erase(rp);
      } else {
        ++rp;
      }
    }
  }

  void timer_stamped_callback()
  {
    auto requeststamped = std::make_shared<practica_msgs::srv::GetPoseStamped::Request>();
    requeststamped->a = "What is the last coordinates received in node_D?";

    pending_responses_stamped_.push_back(client_stamped_->async_send_request(requeststamped));

    auto rp_stamped = pending_responses_stamped_.begin();

    while (rp_stamped != pending_responses_stamped_.end()) {
      if (rp_stamped->valid() && rp_stamped->wait_for(100ms) == std::future_status::ready) {
        auto resp_stamped = rp_stamped->get();
        RCLCPP_INFO(get_logger(), "Node D: [%f, %f, %f]",
          resp_stamped->x, resp_stamped->y, resp_stamped->z);

        rp_stamped = pending_responses_stamped_.erase(rp_stamped);
      } else {
        ++rp_stamped;
      }
    }
  }

private:
  rclcpp::Client<practica_msgs::srv::GetString>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;


  std::list<SharedFuture> pending_responses_;

  rclcpp::Client<practica_msgs::srv::GetPoseStamped>::SharedPtr client_stamped_;
  rclcpp::TimerBase::SharedPtr timer_stamped_;

  std::list<SharedFuturePose> pending_responses_stamped_;
};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node_E = std::make_shared<Client>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_E);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
