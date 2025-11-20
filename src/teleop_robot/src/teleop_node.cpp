#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

// --- Configuration ---
const double MAX_LIN_VEL = 0.5;  // m/s
const double MAX_ANG_VEL = 1.0;  // rad/s
const double STOP_TIMEOUT = 0.5; // Seconds before robot stops if no key is held

// Helper: Non-blocking key read
char get_key()
{
    char buf = 0;
    // CHANGE HERE: Removed "= {0}" to fix the "missing initializer" warning.
    // tcgetattr will fill this struct with data immediately anyway.
    struct termios old; 
    
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    
    if (read(0, &buf, 1) < 0)
        perror("read()");
    
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    
    return buf;
}

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode() : Node("teleop_node")
    {
        // 1. Publisher Setup
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/diff_cont/cmd_vel_unstamped", 10);

        // 2. Timer Setup (Runs at 10Hz / 100ms)
        // This restores your "publish at certain ticks" logic
        timer_ = this->create_wall_timer(
            100ms, std::bind(&TeleopNode::publishLoop, this));

        // Initialize timestamp
        last_key_time_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(this->get_logger(), "Teleop Node Started.");
        RCLCPP_INFO(this->get_logger(), "Hold W/A/S/D to move. Release to stop.");
        RCLCPP_INFO(this->get_logger(), "Press CTRL+C to quit.");

        // 3. Start Input Thread
        input_thread_ = std::thread(&TeleopNode::inputLoop, this);
    }

    ~TeleopNode()
    {
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    // --- The "Ticker" ---
    // Publishes messages at a fixed rate and checks for timeout
    void publishLoop()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        auto now = std::chrono::steady_clock::now();
        double seconds_since_input = std::chrono::duration_cast<std::chrono::duration<double>>(
            now - last_key_time_).count();

        // Watchdog Check: If too much time passed, reset to zero
        if (seconds_since_input > STOP_TIMEOUT) {
            current_twist_.linear.x = 0.0;
            current_twist_.angular.z = 0.0;
        }

        publisher_->publish(current_twist_);
    }

    // --- The "Watcher" ---
    // Blocks waiting for keys, updates velocity, and resets the timer
    void inputLoop()
    {
        char key;
        while (rclcpp::ok())
        {
            key = get_key(); // Blocks here until you press a key

            // Lock before updating shared variables
            {
                std::lock_guard<std::mutex> lock(mutex_);
                
                // Update Timestamp
                last_key_time_ = std::chrono::steady_clock::now();

                // Update Velocity Command
                if (key == 'w') {
                    current_twist_.linear.x = MAX_LIN_VEL;
                    current_twist_.angular.z = 0.0;
                }
                else if (key == 's') {
                    current_twist_.linear.x = -MAX_LIN_VEL;
                    current_twist_.angular.z = 0.0;
                }
                else if (key == 'a') {
                    current_twist_.linear.x = 0.0;
                    current_twist_.angular.z = MAX_ANG_VEL;
                }
                else if (key == 'd') {
                    current_twist_.linear.x = 0.0;
                    current_twist_.angular.z = -MAX_ANG_VEL;
                }
                else if (key == ' ' || key == 'x') {
                    current_twist_.linear.x = 0.0;
                    current_twist_.angular.z = 0.0;
                }
                else if (key == '\x03') { // Ctrl+C
                    rclcpp::shutdown();
                    break;
                }
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread input_thread_;
    
    // Thread-Safety handling
    std::mutex mutex_;
    geometry_msgs::msg::Twist current_twist_;
    std::chrono::steady_clock::time_point last_key_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}