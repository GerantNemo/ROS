#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/*Execute des manoeuvres précédemment enregistrees. Donne la possibilité de passer en manuel par l'appui d'un bouton*/

class GuidageSimple : public rclcpp::Node
{
  public:
    GuidageSimple(): Node("guidage_simple")
    {
      cmd_vel_mode_pub_ = this->create_publisher<std_msgs::msg::Int32>("/SousMarin/command/mode", 10);
      cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/SousMarin/command/pos_vel_pos", 10);

      timer_ = this->create_wall_timer(100ms, std::bind(&GuidageSimple::timer_callback, this));
      timer2_ = this->create_wall_timer(10000ms, std::bind(&GuidageSimple::etat_callback, this));
    }

  private:

    void timer_callback()
    {
        vx = 0.0;
        vy = 0.0;
        vz = 0.0;
        p = 0.0;
        q = 0.0;
        r = 0.0;

        switch(etat)
        {
          case 0: vx = 5.0;
                  break;
          
          case 1: r = 1.0;
                  break;

          case 2: vx = -5.0;
                  break;

          case 3: r = -1.0;
                  break;

          case 4: r = 5.0;
                  break;

          default:r = 0;
        }
        
        cmd_motors.linear.x = vx; 
        cmd_motors.linear.y = vy; 
        cmd_motors.linear.z = vz;
        cmd_motors.angular.x = p; 
        cmd_motors.angular.y = q; 
        cmd_motors.angular.z = r;

        cmd_vel_pub_->publish(cmd_motors);
        
        //Guidage mode : 1 : mauel ; 0 : automatique
        cmd_vel_mode.data = 0;
        cmd_vel_mode_pub_->publish(cmd_vel_mode);
    }

    void etat_callback()
    {
      if(etat == 5)
        etat = 5;
      else
        etat++;
    }

    //Subscribers and publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cmd_vel_mode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    //Messages
    std_msgs::msg::Int32 cmd_vel_mode;
    geometry_msgs::msg::Twist cmd_motors;

    //Variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer2_;

    int etat = 0;

    float vx;
    float vy;
    float vz;
    float p;
    float q;
    float r;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuidageSimple>());
  rclcpp::shutdown();
  return 0;
}