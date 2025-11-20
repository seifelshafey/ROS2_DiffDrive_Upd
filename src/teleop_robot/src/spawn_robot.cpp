#include <functional>
#include <memory>
#include<algorithm>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::placeholders;

class SpawnService : public rclcpp::Node{

    public:
    //Calls service node
        SpawnService() : Node("spawn_node"){
            //Default origin
            this->declare_parameter<std::vector<double>>("origin", {0.0,0.0,0.0});
            //Spawn flag
            this->declare_parameter("robot_spawned", false);
            //service to be called upon boot up
            server_ = this->create_service<std_srvs::srv::Empty>("spawn", std::bind(&SpawnService::service_callback, this, _1, _2));
            spawn_transform = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        }
    private:
        void service_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response){
            (void)request;
            (void)response;
            //flags robot as spawned
            this->set_parameter(rclcpp::Parameter("robot_spawned", true));
            //Updates origin parameter if provided
            std::vector<double> og;
            if (!this->get_parameter("origin", og) || std::all_of(og.begin(), og.end(), [](double x) { return x == 0.0; })) 
            {
                RCLCPP_INFO(this->get_logger(), "Default Origin Used");
                og =  std::vector<double>{0, 0, 0};
            }
            RCLCPP_INFO(this->get_logger(),"Spawning robot at x = %f, y = %f, theta = %f", og[0],  og[1],  og[2]);
            //Creates static transform from world to robot
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "world";
            t.child_frame_id = "base_link";

            t.transform.translation.x = og[0];
            t.transform.translation.y = og[1];
            tf2::Quaternion q;
            q.setRPY(0 , 0, og[2]);
            t.transform.rotation.x = 0;
            t.transform.rotation.y = 0;
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            //Published static transform
            spawn_transform->sendTransform(t);
        }
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> spawn_transform;
        geometry_msgs::msg::TransformStamped t;
};

int main(int argc, char * argv[])
{
  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpawnService>());
  rclcpp::shutdown();
  return 0;
}