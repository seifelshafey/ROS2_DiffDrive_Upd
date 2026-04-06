#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdio>
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
const double MAX_LIN_VEL = 1.0;   // m/s
const double MAX_ANG_VEL = 0.75;  // rad/s

// Time (seconds) before an axis decays to zero after the last keypress.
// Must exceed macOS key repeat delay (~500ms) to prevent stutter when holding keys.
// Tradeoff: a single tap causes the robot to coast for this duration before stopping.
// Use Space/X for immediate emergency stop.
const double STOP_TIMEOUT = 0.8;

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode() : Node("teleop_node")
    {
        // --- Terminal Setup (once) ---
        // Save original terminal settings so we can restore them on shutdown.
        // These settings only affect this xterm's pty, not other terminals.
        tcgetattr(STDIN_FILENO, &original_termios_);

        struct termios raw = original_termios_;
        raw.c_lflag &= ~(ICANON | ECHO | ISIG);  // Disable line buffering, echo, and signal generation
        raw.c_iflag &= ~ICRNL;  // Don't translate CR to NL (raw Enter key)
        raw.c_cc[VMIN] = 0;   // Don't require any minimum characters
        raw.c_cc[VTIME] = 1;  // 100ms read timeout (allows clean shutdown checks)
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        // --- Publisher Setup ---
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/diff_cont/cmd_vel_unstamped", 10);

        // --- Timer Setup (10Hz / 100ms) ---
        // Each tick: check per-axis decay timers, then publish current velocity.
        timer_ = this->create_wall_timer(
            100ms, std::bind(&TeleopNode::publishLoop, this));

        // Initialize per-axis timestamps to epoch (triggers immediate decay to zero)
        last_linear_key_time_ = std::chrono::steady_clock::time_point{};
        last_angular_key_time_ = std::chrono::steady_clock::time_point{};

        printHelp();

        // --- Start Input Thread ---
        input_thread_ = std::thread(&TeleopNode::inputLoop, this);
    }

    ~TeleopNode()
    {
        // Signal the input thread to stop, then wait for it to exit.
        // MUST join BEFORE restoring termios -- otherwise tcsetattr re-enables
        // canonical mode while the input thread is still in read(), causing
        // read() to block forever waiting for a newline. Deadlock.
        stop_ = true;
        if (input_thread_.joinable()) {
            input_thread_.join();  // Thread exits within one VTIME interval (100ms)
        }

        // Now safe to restore terminal settings (no thread is reading stdin).
        tcsetattr(STDIN_FILENO, TCSADRAIN, &original_termios_);
    }

private:
    void printHelp()
    {
        printf("\n");
        printf("  Teleop Keyboard Controls\n");
        printf("  ========================\n");
        printf("        W          (forward)\n");
        printf("   Q    ^    E     (diagonal shortcuts)\n");
        printf("   A  < x >  D    (turn left / right)\n");
        printf("   Z    v    C     (diagonal shortcuts)\n");
        printf("        S          (reverse)\n");
        printf("\n");
        printf("  SPACE/X : Emergency stop\n");
        printf("  CTRL+C  : Quit\n");
        printf("\n");
        printf("  Hold two keys (W+D) for diagonal movement.\n");
        printf("  Each axis decays independently after release.\n");
        printf("\n");
    }

    // Read one character from stdin. Returns 0 if no key is available (100ms timeout).
    // Terminal is already in raw mode -- no termios calls needed here.
    char getKey()
    {
        char buf = 0;
        ssize_t n = read(STDIN_FILENO, &buf, 1);
        if (n <= 0) return 0;
        return buf;
    }

    // --- Publish Loop (10Hz timer callback) ---
    // Checks per-axis decay timers and publishes the current velocity.
    void publishLoop()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        auto now = std::chrono::steady_clock::now();

        // Each axis decays to zero independently after STOP_TIMEOUT seconds
        // since its last keypress. This allows W to stay active while D decays.
        double lin_elapsed = std::chrono::duration<double>(
            now - last_linear_key_time_).count();
        double ang_elapsed = std::chrono::duration<double>(
            now - last_angular_key_time_).count();

        if (lin_elapsed > STOP_TIMEOUT) {
            current_twist_.linear.x = 0.0;
        }
        if (ang_elapsed > STOP_TIMEOUT) {
            current_twist_.angular.z = 0.0;
        }

        publisher_->publish(current_twist_);

        // Live velocity display (overwrites same line via carriage return)
        printf("\r  lin: %+5.2f m/s  |  ang: %+5.2f rad/s  ",
               current_twist_.linear.x, current_twist_.angular.z);
        fflush(stdout);
    }

    // --- Input Loop (dedicated thread) ---
    // Reads keys and updates velocity. Each key only modifies its own axis,
    // enabling diagonal movement when two keys are held simultaneously.
    void inputLoop()
    {
        while (rclcpp::ok() && !stop_.load())
        {
            char raw_key = getKey();
            if (raw_key == 0) continue;  // Timeout, no key in buffer

            // Normalize to lowercase so Caps Lock / Shift don't break controls
            char key = static_cast<char>(std::tolower(static_cast<unsigned char>(raw_key)));

            std::lock_guard<std::mutex> lock(mutex_);
            auto now = std::chrono::steady_clock::now();

            switch (key) {
                // --- Linear axis only (W/S) ---
                case 'w':
                    current_twist_.linear.x = MAX_LIN_VEL;
                    last_linear_key_time_ = now;
                    break;
                case 's':
                    current_twist_.linear.x = -MAX_LIN_VEL;
                    last_linear_key_time_ = now;
                    break;

                // --- Angular axis only (A/D) ---
                case 'a':
                    current_twist_.angular.z = MAX_ANG_VEL;
                    last_angular_key_time_ = now;
                    break;
                case 'd':
                    current_twist_.angular.z = -MAX_ANG_VEL;
                    last_angular_key_time_ = now;
                    break;

                // --- Diagonal shortcuts (both axes) ---
                case 'q':  // Forward + left turn
                    current_twist_.linear.x = MAX_LIN_VEL;
                    current_twist_.angular.z = MAX_ANG_VEL;
                    last_linear_key_time_ = now;
                    last_angular_key_time_ = now;
                    break;
                case 'e':  // Forward + right turn
                    current_twist_.linear.x = MAX_LIN_VEL;
                    current_twist_.angular.z = -MAX_ANG_VEL;
                    last_linear_key_time_ = now;
                    last_angular_key_time_ = now;
                    break;
                case 'z':  // Reverse + left turn
                    current_twist_.linear.x = -MAX_LIN_VEL;
                    current_twist_.angular.z = MAX_ANG_VEL;
                    last_linear_key_time_ = now;
                    last_angular_key_time_ = now;
                    break;
                case 'c':  // Reverse + right turn
                    current_twist_.linear.x = -MAX_LIN_VEL;
                    current_twist_.angular.z = -MAX_ANG_VEL;
                    last_linear_key_time_ = now;
                    last_angular_key_time_ = now;
                    break;

                // --- Emergency stop ---
                case ' ':
                case 'x':
                    current_twist_.linear.x = 0.0;
                    current_twist_.angular.z = 0.0;
                    // Set timestamps to epoch so decay triggers immediately
                    last_linear_key_time_ = std::chrono::steady_clock::time_point{};
                    last_angular_key_time_ = std::chrono::steady_clock::time_point{};
                    break;

                // --- Quit ---
                // With ISIG disabled, Ctrl+C arrives as raw byte 0x03 instead of
                // generating SIGINT. We handle shutdown explicitly here.
                case '\x03':  // Ctrl+C
                    current_twist_.linear.x = 0.0;
                    current_twist_.angular.z = 0.0;
                    publisher_->publish(current_twist_);
                    rclcpp::shutdown();
                    return;
            }
        }
    }

    // --- ROS2 members ---
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread input_thread_;

    // --- Terminal state ---
    struct termios original_termios_;

    // --- Thread-safe shared state ---
    std::atomic<bool> stop_{false};  // Signals the input thread to exit
    std::mutex mutex_;
    geometry_msgs::msg::Twist current_twist_;

    // Per-axis timestamps for independent decay.
    // When a key is pressed, only its axis timestamp is refreshed.
    // Each axis decays to zero independently after STOP_TIMEOUT.
    std::chrono::steady_clock::time_point last_linear_key_time_;
    std::chrono::steady_clock::time_point last_angular_key_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}