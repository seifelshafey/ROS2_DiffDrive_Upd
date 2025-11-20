#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::placeholders;

class State_Estimation : public rclcpp::Node{

    public: 
        State_Estimation() : Node("state_est"){
            subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_tele", 10,
                std::bind(&State_Estimation::state_update, this, _1));
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            state_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "teleop_node");

            if (!param_client_->wait_for_service(std::chrono::milliseconds(100)))
            {
                RCLCPP_ERROR(this->get_logger(), "Parameter service not available");
                return;
            }
            auto future = param_client_->get_parameters({"rate"});
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
            {
            auto results = future.get();
            auto param = results.at(0);
           RCLCPP_INFO(this->get_logger(), "Received parameter value: %s",
                  param.value_to_string().c_str());
            }
           else {
                RCLCPP_ERROR(this->get_logger(), "Failed to get parameter");
            }
            last_update_time_ = this->now();
    }

    private:
        void state_update(const geometry_msgs::msg::Twist::SharedPtr vel_upd)
        {
            auto now = this->now();
            if (now.get_clock_type() != last_update_time_.get_clock_type()) {
    RCLCPP_ERROR(this->get_logger(), "Clock types differ! now=%d, last_update_time_=%d",
                 now.get_clock_type(), last_update_time_.get_clock_type());
    last_update_time_ = now; // Reset last update
    return; // Skip update
}
            double dt = (now - last_update_time_).seconds();
            last_update_time_ = now;
            if (dt <= 0.0) dt = rate;
            geometry_msgs::msg::TransformStamped t;
            auto upd_t = geometry_msgs::msg::TransformStamped();
            tf2::Quaternion n;
            try {
                t = tf_buffer_->lookupTransform("world","base_link", tf2::TimePointZero);
            } 
            catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(this->get_logger(), "Could not transform robot to world: %s", ex.what());
                return;
            }
            upd_t.header.stamp = this->get_clock()->now();
            upd_t.header.frame_id = "world";  
            upd_t.child_frame_id = "base_link";

            tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            upd_t.transform.translation.x = t.transform.translation.x + (vel_upd->linear.x * cos(yaw)* dt);
            upd_t.transform.translation.y = t.transform.translation.y + (vel_upd->linear.x * sin(yaw)* dt);
            upd_t.transform.translation.z = t.transform.translation.z;
            new_yaw = yaw + (vel_upd->angular.z*dt);
            new_yaw = new_yaw - 2.0 * M_PI * std::floor((new_yaw + M_PI) / (2.0 * M_PI));

            n.setRPY(roll, pitch,new_yaw);
            n.normalize();
            upd_t.transform.rotation = tf2::toMsg(n);

            state_broadcaster_->sendTransform(upd_t);
        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> state_broadcaster_;
        rclcpp::AsyncParametersClient::SharedPtr param_client_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::Time last_update_time_;
        double rate = 0;
        double roll, pitch, yaw;
        double new_yaw;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<State_Estimation>());
  rclcpp::shutdown();
  return 0;
}