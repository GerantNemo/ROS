#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

/*Se contente d'informer le pilotage que le mode est manuel : le pilotage ecoute alors la manette playstation (Dualshock 4) ou le clavier */

class GuidageBase : public rclcpp::Node
{
  public:
    GuidageBase(): Node("guidage_base")
    {
      cmd_vel_mode_pub_ = this->create_publisher<std_msgs::msg::Int32>("/SousMarin/command/mode", 10);
      timer_ = this->create_wall_timer(100ms, std::bind(&GuidageBase::timer_callback, this));
    }

  private:

    void timer_callback()
    {
        //Guidage mode : 1 : mauel ; 0 : automatique
        cmd_vel_mode.data = 1;
        cmd_vel_mode_pub_->publish(cmd_vel_mode);
    }

    //Subscribers and publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cmd_vel_mode_pub_;
    
    //Messages
    std_msgs::msg::Int32 cmd_vel_mode;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuidageBase>());
  rclcpp::shutdown();
  return 0;
}