#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

/*Execute des manoeuvres précédemment enregistrees. Donne la possibilité de passer en manuel par l'appui d'un bouton
Plonger, effectuer un scan, remonter, avec des correcteurs P pour la profondeur*/

class GuidageSonar : public rclcpp::Node
{
  public:
    GuidageSonar(): Node("guidage_sonar1")
    {
      sonar_pos_init_sub_= this->create_subscription<geometry_msgs::msg::Vector3>("/SousMarin/Sonar/Pos_Init", 10, std::bind(&GuidageSonar::callback_pos_init, this, _1));
      sonar_pos_cible_sub_= this->create_subscription<geometry_msgs::msg::Vector3>("/SousMarin/Sonar/Pos_Cible", 10, std::bind(&GuidageSonar::callback_pos_cible, this, _1));
      sonar_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>("/SousMarin/Sonar/Angle", 10, std::bind(&GuidageSonar::callback_angle, this, _1));

      requete_pub_ = this->create_publisher<std_msgs::msg::Int32>("/SousMarin/Sonar/Requete", 10);
      
      cmd_vel_mode_pub_ = this->create_publisher<std_msgs::msg::Int32>("/SousMarin/command/mode", 10);
      cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/SousMarin/command/pos_vel_pos", 10);
      cmd_pos_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/SousMarin/command/pos", 10);

      bloquage_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/SousMarin/command/bloquage", 10);

      timer_ = this->create_wall_timer(100ms, std::bind(&GuidageSonar::timer_callback, this));
    }

    void bloquage_cmd(float bl_vx, float bl_vy, float bl_vz, float bl_p, float bl_q, float bl_r)
    {
      geometry_msgs::msg::Twist bloquage;
      bloquage.linear.x = bl_vx;
      bloquage.linear.y = bl_vy;
      bloquage.linear.z = bl_vz;
      bloquage.angular.x = bl_p;
      bloquage.angular.y = bl_q;
      bloquage.angular.z = bl_r;
      RCLCPP_INFO(this->get_logger(), "Bloquage : %lf, %lf, %lf, %lf, %lf, %lf", bloquage.linear.x, bloquage.linear.y, bloquage.linear.z,
                      bloquage.angular.x , bloquage.angular.y , bloquage.angular.z );
                      
      bloquage_pub_->publish(bloquage);
    }

    void send_vit(float vx, float vy, float vz, float p, float q, float r)
    {
      geometry_msgs::msg::Twist vitesse;
      vitesse.linear.x = vx;
      vitesse.linear.y = vy;
      vitesse.linear.z = vz;
      vitesse.angular.x = p;
      vitesse.angular.x = q;
      vitesse.angular.x = r;
      cmd_vel_pub_->publish(vitesse);
    }

    void profondeur(float prof, int etat_suivant)
    {
        RCLCPP_INFO(this->get_logger(), "Profondeur en cours");
        
        if(start_fonc == 0)
        {
          start_fonc = 1;
          start_time = rclcpp::Clock{RCL_ROS_TIME}.now();
          dt = 0;
        }

        if(abs(pos.pose.pose.position.z-prof)>0.1 && dt <= 20.0)
        {
          RCLCPP_INFO(this->get_logger(), "Verif");
          
          cmd_vel_mode.data = 0; //commandes position
          cmd_vel_mode_pub_->publish(cmd_vel_mode);

          geometry_msgs::msg::Pose cmd_pos;

          cmd_pos.position.x = 0.0;
          cmd_pos.position.y = 0.0;
          cmd_pos.position.z = prof;
          cmd_pos.orientation.x = 0.0;
          cmd_pos.orientation.y = 0.0;
          cmd_pos.orientation.z = 0.0;
          cmd_pos.orientation.w = 0.0;

          bloquage_cmd(1.0, 1.0, 0.0, 1.0, 1.0, 1.0);
          cmd_pos_pub_->publish(cmd_pos);

          current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
          dt = current_time.seconds() - start_time.seconds();

          //Bloquage des commandes
          //bloquage_cmd(1.0, 1.0, 0.0, 1.0, 1.0, 1.0);
        }
        else
        {
          etat = etat_suivant;
          start_fonc = 0;
        }

        //using namespace std::this_thread; // sleep_for, sleep_until
        //using namespace std::chrono; // nanoseconds, system_clock, seconds

        //sleep_for(nanoseconds(10));
        //std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(10));
    }

    void arret(int etat_suivant)
    {
        RCLCPP_INFO(this->get_logger(), "Arret en cours");
        
        cmd_vel_mode.data = 0; //commandes position
        cmd_vel_mode_pub_->publish(cmd_vel_mode);

        geometry_msgs::msg::Twist cmd_vel;

        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;

        bloquage_cmd(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
        cmd_vel_pub_->publish(cmd_vel);

        //Bloquage des commandes
        //bloquage_cmd(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);

        //using namespace std::this_thread; // sleep_for, sleep_until
        //using namespace std::chrono; // nanoseconds, system_clock, seconds

        //sleep_for(nanoseconds(10));
        //std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(10));

        etat = etat_suivant;
    }

    void demande_scan(int etat_suivant)
    {   
      RCLCPP_INFO(this->get_logger(), "Demande scan en cours");
      
      //bloquage_cmd(1.0, 1.0, 0.0, 1.0, 1.0, 1.0);
      //send_vit(0.0, 0.0, vit_prof_ref, 0.0, 0.0, 0.0);
      
      if(start_fonc == 0)
      {
            if(nbre_scan >= 3)
                etat = 3;
            
            std_msgs::msg::Int32 req;
            req.data = 1;
            requete_pub_->publish(req);
            start_fonc = 1;
            nbre_scan ++;

            rec_pos_init = 0;
            rec_pos_cible = 0;
            rec_angle = 0;
            
            last_time = rclcpp::Clock{RCL_ROS_TIME}.now();
            dt = 0.0;
        }
        
        if(dt <= 500)
        {
          current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
          dt = current_time.seconds() - last_time.seconds();

          geometry_msgs::msg::Pose cmd_pos;

          cmd_pos.position.x = 0.0;
          cmd_pos.position.y = 0.0;
          cmd_pos.position.z = 1.0;
          cmd_pos.orientation.x = 0.0;
          cmd_pos.orientation.y = 0.0;
          cmd_pos.orientation.z = 0.0;
          cmd_pos.orientation.w = 0.0;

          bloquage_cmd(1.0, 1.0, 0.0, 1.0, 1.0, 0.0);
          //cmd_pos_pub_->publish(cmd_pos);
          bloquage_cmd(1.0, 1.0, 0.0, 1.0, 1.0, 0.0);
          //send_vit(0.0, 0.0, vit_prof_ref, 0.0, 0.0, 0.0);
        }
        else
        {
            etat = 4;
            start_fonc = 0;
        }
        
        if(rec_pos_init  == 1 && rec_pos_cible == 1 && rec_angle == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Scan effectue");
            etat = etat_suivant;
            start_fonc = 0;
        }
    }
    
    void tourner(int etat_suivant)
    {
        profondeur(prof_ref, etat_suivant);
        
        RCLCPP_INFO(this->get_logger(), "Tourner en cours");
        
        cmd_vel_mode.data = 1; //commandes vitesses
        cmd_vel_mode_pub_->publish(cmd_vel_mode);

        if(start_fonc == 0)
        {
          start_fonc = 1;
          last_time = rclcpp::Clock{RCL_ROS_TIME}.now();
          dt = 0.0;

          //estimation temps necessaire
          duree = angle.data * vit_rad_ref;
        }

        if(dt <= duree)
        {
          current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
          dt = current_time.seconds() - last_time.seconds();

          bloquage_cmd(1.0, 1.0, 0.0, 1.0, 1.0, 0.0);
          send_vit(0.0, 0.0, vit_prof_ref, 0.0, 0.0, vit_rad_ref * sign(angle.data));
          //bloquage_cmd(1.0, 1.0, 0.0, 1.0, 1.0, 0.0);
        }
        else
        {
          etat = etat_suivant;
          start_fonc = 0;
        }
    }

    void aller(int etat_suivant, int etat_suivant2)
    {
        profondeur(prof_ref, etat_suivant);
        
        RCLCPP_INFO(this->get_logger(), "Aller en cours");
        
        cmd_vel_mode.data = 1; //commandes vitesses
        cmd_vel_mode_pub_->publish(cmd_vel_mode);

        if(start_fonc == 0)
        {
          start_fonc = 1;
          last_time = rclcpp::Clock{RCL_ROS_TIME}.now();
          dt = 0.0;

          //estimation temps necessaire
          double distance = sqrt(pow((pos_init.x - pos_cible.x), 2) + pow((pos_init.y - pos_cible.y), 2));
          duree = distance * vit_ref;

          if(distance < 0.5 || nbre_aller >= 5)
            etat = etat_suivant2;
        }

        if(dt <= duree)
        {
          current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
          dt = current_time.seconds() - last_time.seconds();

          bloquage_cmd(0.0, 1.0, 0.0, 1.0, 1.0, 1.0);
          send_vit(0.0, 0.0, vit_prof_ref, 0.0, 0.0, vit_ref);
          //bloquage_cmd(0.0, 1.0, 0.0, 1.0, 1.0, 1.0);

        }
        else
        {
          etat = etat_suivant;
          start_fonc = 0;
          nbre_aller ++;
        }
    }

    template <typename T> int sign(T val) 
    {
        return (T(0) < val) - (val < T(0));
    }

  private:
    
    void timer_callback()
    {
        double prof_ref = 1.0;
        RCLCPP_INFO(this->get_logger(), "Etat : %i", etat);
        
        switch(etat)
        {
          case 0: profondeur(prof_ref, 1);
                  break;
          
          case 1: demande_scan(2);
                  break;

          case 2: tourner(3);
                  break;

          case 3: aller(2, 4);
                  break;

          case 4: profondeur(0.0, 5);
                  break;

          case 5: arret(5);
                  break;

          default: arret(5);
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

    void callback_angle(const std_msgs::msg::Float64& angle_msg)
    {
        angle = angle_msg;
        rec_angle = 1;
    }

    void callback_pose(const nav_msgs::msg::Odometry& pos_msg)
    {
        pos = pos_msg;
    }

    //Subscribers and publishers
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sonar_pos_init_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sonar_pos_cible_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sonar_angle_sub_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr requete_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cmd_vel_mode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr bloquage_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr cmd_pos_pub_;
    
    //Messages
    std_msgs::msg::Int32 cmd_vel_mode;
    geometry_msgs::msg::Twist cmd_vel;
    geometry_msgs::msg::Vector3 pos_init;
    geometry_msgs::msg::Vector3 pos_cible;
    std_msgs::msg::Float64 angle;
    nav_msgs::msg::Odometry pos;

    //Time
    rclcpp::Time last_time;
    rclcpp::Time current_time;
    rclcpp::Time start_time;
    double dt;
    double duree;

    //Positions
    double x_init, y_init, z_init;
    double vx_init, vy_init, vz_init;

    double roll_init, pitch_init, yaw_init;
    double p_init, q_init, r_init;

    //Variables
    rclcpp::TimerBase::SharedPtr timer_;

    //references
    double vit_ref = 2; //m/s
    double vit_prof_ref = 1.0;
    double vit_rad_ref = 2; //rad/s

    int etat = 0;
    
    int prof_ref = 1.0;

    int aller_debut = 0;

    int rec_pos_init = 0;
    int rec_pos_cible = 0;
    int rec_angle = 0;

    int nbre_scan = 0;
    int nbre_aller = 0;

    int start_fonc = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuidageSonar>());
  rclcpp::shutdown();
  return 0;
}
