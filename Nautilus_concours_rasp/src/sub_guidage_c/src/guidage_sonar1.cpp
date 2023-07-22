#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

/*Execute des manoeuvres précédemment enregistrees. (Donne la possibilité de passer en manuel par l'appui d'un bouton)
AVancer et tourner*/

class GuidageSonar : public rclcpp::Node
{
  public:
    GuidageSonar(): Node("guidage_sonar1")
    {
      requete_pub_ = this->create_publisher<std_msgs::msg::Int32>("/SousMarin/Sonar/Requete", 10);
      
      cmd_vel_mode_pub_ = this->create_publisher<std_msgs::msg::Int32>("/SousMarin/command/mode", 10);
      cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/SousMarin/command/pos_vel_pos", 10);
      cmd_pos_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/SousMarin/command/pos", 10);

      mes_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/model/SousMarin/odometry", 10);

      timer_ = this->create_wall_timer(100ms, std::bind(&GuidageSonar::timer_callback, this));
    }


    void plongee()
    {
        cmd_vel_mode.data = 1; //commandes vitesse
        cmd_vel_mode_pub_->publish(cmd_vel_mode);

        geometry_msgs::msg::Twist cmd_vel;

        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = -10.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;

        cmd_vel_pub_->publish(cmd_vel);

        //using namespace std::this_thread; // sleep_for, sleep_until
        //using namespace std::chrono; // nanoseconds, system_clock, seconds

        //sleep_for(nanoseconds(10));
        std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(10));

        etat = 1;
    }

    void remontee()
    {
        cmd_vel_mode.data = 1; //commandes vitesse
        cmd_vel_mode_pub_->publish(cmd_vel_mode);

        geometry_msgs::msg::Twist cmd_vel;

        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 10.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;

        cmd_vel_pub_->publish(cmd_vel);

        std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(10));
    }

    void demande_scan()
    {   
        //Arret du sous-marin
        cmd_vel_mode.data = 1; //commandes vitesse
        cmd_vel_mode_pub_->publish(cmd_vel_mode);

        geometry_msgs::msg::Twist cmd_vel;

        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;

        cmd_vel_pub_->publish(cmd_vel);

        if(debut_scan == 0)
        {
            if(nbre_scan >= 3)
              etat = 3;
            
            std_msgs::msg::Int32 req;
            req.data = 1;
            requete_pub_->publish(req);
            debut_scan = 1;
            nbre_scan ++;
        }

        if(rec_pos_init  == 1 && rec_pos_cible == 1)
        {
            fin_scan = 1;
        }

        if(debut_scan == 1 && fin_scan == 1)
        {
            double d = sqrt(pow(pos_init.x - pos_cible.x, 2) + pow(pos_init.y -pos_cible.y, 2) + pow(pos_init.z - pos_cible.z, 2));
            if(d <= 2.0)
            {
              etat = 3;
            }
            else
            {
              etat = 2;
            }
            debut_scan = 0;
            fin_scan = 0;
            rec_pos_init = 0;
            rec_pos_cible = 0;

            pos_est.pose.pose.position.x = pos_init.x;
            pos_est.pose.pose.position.y = pos_init.y;
            pos_est.pose.pose.position.z = pos_init.z;
            pos_est.pose.pose.orientation.x = 0.0;
            pos_est.pose.pose.orientation.y = 0.0;
            pos_est.pose.pose.orientation.z = 0.0;
            pos_est.pose.pose.orientation.w = 0.0;

            cmd_pos.position.x = pos_cible.x;
            cmd_pos.position.y = pos_cible.y;
            cmd_pos.position.z = pos_cible.z;
            cmd_pos.orientation.x = 0.0;
            cmd_pos.orientation.y = 0.0;
            cmd_pos.orientation.z = 0.0;
            cmd_pos.orientation.w = 0.0;
        }
    }

    void aller_pos()
    {
        cmd_vel_mode.data = 0; //commandes position
        cmd_vel_mode_pub_->publish(cmd_vel_mode);
        
        if(aller_debut == 0)
        {
          aller_debut = 1;
          last_time = rclcpp::Clock{RCL_ROS_TIME}.now();
        }
        else
        {
          /*current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
          dt = current_time.seconds() - last_time.seconds();
          last_time = current_time;
          if (dt == 0.0) return;

          //Mettre la vitesse actuelle
          cmd_pose.point.x = 1.0*dt;
          cmd_pose.point.y = 1.0*dt;
          cmd_pose.point.z = 1.0*dt;
          cmd_pose.quaternion.x = 0.0;
          cmd_pose.quaternion.y = 0.0;
          cmd_pose.quaternion.z = 0.0;
          cmd_pose.quaternion.w = 0.0;*/

          std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(60));
        }
        
        mes_odom_pub_->publish(pos_est);
        cmd_pos_pub_->publish(cmd_pos);

        aller_debut = 0;
        etat = 1;
    }

  private:

    void timer_callback()
    {
        switch(etat)
        {
          case 0: plongee();
                  break;
          
          case 1: demande_scan();
                  break;

          case 2: aller_pos();
                  break;

          case 3: remontee();
                  break;

          default: remontee();
        }
    }

    void callback_pos_init(const geometry_msgs::msg::Vector3& pos_init_msg)
    {
        pos_init = pos_init_msg;
        rec_pos_init = 1;
    }

    void callback_pos_cible(const geometry_msgs::msg::Vector3& pos_cible_msg)
    {
        pos_cible = pos_cible_msg;
        rec_pos_cible = 1;
    }

    //Subscribers and publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr requete_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cmd_vel_mode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr cmd_pos_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mes_odom_pub_;
    
    //Messages
    std_msgs::msg::Int32 cmd_vel_mode;
    geometry_msgs::msg::Twist cmd_vel;
    geometry_msgs::msg::Pose cmd_pos;
    geometry_msgs::msg::Vector3 pos_init;
    geometry_msgs::msg::Vector3 pos_cible;
    nav_msgs::msg::Odometry pos_est;

    //Time
    rclcpp::Time last_time;
    rclcpp::Time current_time;
    double dt;

    //Variables
    rclcpp::TimerBase::SharedPtr timer_;

    int etat = 0;

    int debut_scan = 0;
    int fin_scan = 0;

    int aller_debut = 0;

    int rec_pos_init = 0;
    int rec_pos_cible = 0;

    int nbre_scan = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuidageSonar>());
  rclcpp::shutdown();
  return 0;
}