#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <random>
#include <vector>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "rclcpp/rclcpp.hpp"
#include "serial_test1/ArduinoJson-v6.21.2.hpp"

#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

class Serial_com : public rclcpp::Node
{
  public:
    Serial_com() : Node("serial_com")
    {
        //Subscriber
        cmd_motors_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>("/SousMarin/command/pwm", 10, std::bind(&Serial_com::ControlCallback, this, _1));
        
        //Publisher
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/SousMarin/measured/Imu", 10);
        magnetic_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/SousMarin/measured/MagneticField", 10);
        angle_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/SousMarin/measured/Angle", 10);
        pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/SousMarin/measured/Pressure", 10);
        depth_pub_ = this->create_publisher<std_msgs::msg::Float64>("/SousMarin/measured/Depth", 10);
        temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/SousMarin/measured/Temperature", 10);
        humidity_pub_ = this->create_publisher<sensor_msgs::msg::RelativeHumidity>("/SousMarin/measured/Humidity", 10);
        tension_pub_ = this->create_publisher<std_msgs::msg::Float64>("/SousMarin/measured/Tension", 10);
        
        timer_ = this->create_wall_timer(100ms, std::bind(&Serial_com::timer_callback, this));
        
        init_params();
        open_port();
    }

    //Recuperation de parametres
    void init_params()
    {
        //Declaration de parametres
        this->declare_parameter("Port", "/dev/ttyUSB0");
        this->declare_parameter("Baud_rate", 230400);

        //Recuperation des parametres
        //this->get_parameter("Port", port_name);
        this->get_parameter("Baud_rate", baud_rate);

        cmd_motors.velocities.push_back(1500);
        cmd_motors.velocities.push_back(1500);
        cmd_motors.velocities.push_back(1500);
        cmd_motors.velocities.push_back(1500);
        cmd_motors.velocities.push_back(1500);
        cmd_motors.velocities.push_back(1500);
        cmd_motors.velocities.push_back(1500);
        cmd_motors.velocities.push_back(1500);

    }

    //Ouverture et configuration d'un port
    void open_port()
    {
        //serial_port = open(port_name, O_RDWR);
        serial_port = open( port_name, O_RDWR| O_NOCTTY );
        
        memset(&tty, 0, sizeof tty);

        // Check for errors
        if (serial_port < 0) 
        {
            printf("Error %i from open: %s\n", errno, strerror(errno));
        }
        // Read in existing settings, and handle any error
        // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
        // must have been initialized with a call to tcgetattr() overwise behaviour
        // is undefined
        if(tcgetattr(serial_port, &tty) != 0) 
        {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        }

        /*Save old tty parameters */
        tty_old = tty;
        
        //Control modes
        tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
        //tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
        
        tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
        //tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication
        
        tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
        //tty.c_cflag |= CS5; // 5 bits per byte
        //tty.c_cflag |= CS6; // 6 bits per byte
        //tty.c_cflag |= CS7; // 7 bits per byte
        tty.c_cflag |= CS8; // 8 bits per byte (most common)

        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
        //tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control

        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        //Local modes
        tty.c_lflag &= ~ICANON;

        tty.c_lflag &= ~ECHO; // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo

        tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

        //Input modes
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl

        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

        //Output modes
        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
        // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

        tty.c_cc[VTIME] = 0.001;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        //Baud Rate
        // Set in/out baud rate to be 9600
        //Frequence visee : 230400
        cfsetispeed(&tty, (speed_t)B230400);
        cfsetospeed(&tty, (speed_t)B230400);

        /* Make raw */
        cfmakeraw(&tty);

        /* Flush Port, then applies attributes */
        tcflush(serial_port, TCIFLUSH );
        // Save tty settings, also checking for error
        if (tcsetattr(serial_port, TCSANOW, &tty) != 0) 
        {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        }

        RCLCPP_INFO(this->get_logger(), "Port ouvert");
    }

    //Fermeture du port
    void close_port()
    {
        close(serial_port);
    }

    //Generation d'un nombre aleatoire
    int generate_random_nb()
    {
        // Seed with a real random value, if available
        std::random_device r;
 
        // Choose a random mean between 1 and 20
        std::default_random_engine e1(r());
        std::uniform_int_distribution<int> uniform_dist(1, 20);
        int nb = uniform_dist(e1) * 100;
        return nb;
    }

    //Creation et envoi d'un JSON
    void serialize_json()
    {
        //RCLCPP_INFO(this->get_logger(), "Check1");
        
        StaticJsonDocument<capacity_sent> doc_sent;
        
        std::vector<std::string> keys = {"M1", "M2", "M3", "M4", "MP1", "MP2", "MP3", "MP4"};

        //RCLCPP_INFO(this->get_logger(), "Check2");

        //TODO : set modes automatically
        doc_sent["mode"].set("A");
        doc_sent["type"].set("A");
        doc_sent["op"].set("N");

        for(long unsigned int i=0; i<keys.size(); i++)
        {
            doc_sent[keys[i]].set(cmd_motors.velocities[i]);
        }

        /*for(int i=0; i<8; i++)
        {
            int nb = generate_random_nb();
            doc_sent[keys[i]] = nb;
        }*/

        char msg[256];
        
        serializeJson(doc_sent, msg);

        //RCLCPP_INFO(this->get_logger(), "Envoi du message, taille du msg: %i ", (int)sizeof(msg));
        RCLCPP_INFO(this->get_logger(), "msg: %s ", msg);

        write(serial_port, msg, sizeof(msg));
    }

    //Reception et lecture d'un JSON
    void deserialize_json_trash()
    {
        char read_buf [max_size_t];
        char msg [size_msg_t+1];

        bool verif = 0;

        memset(read_buf, '\0', sizeof read_buf);
        // Read bytes. The behaviour of read() (e.g. does it block?,
        // how long does it block for?) depends on the configuration
        // settings above, specifically VMIN and VTIME
        int n = read(serial_port, &read_buf, sizeof(read_buf));
        
        if (n < 0) {
            RCLCPP_INFO(this->get_logger(), "Error reading: %s", strerror(errno));
        }
        else if (n == 0) {
            RCLCPP_INFO(this->get_logger(), "Read nothing!");
        }
        else {

            StaticJsonDocument<capacity_got> doc_got;
            for(int i=0; i<max_size_t; i++)
            {
                if(read_buf[i] == '{' && (i+size_msg_t)<max_size_t && verif == 0)
                {
                    verif = 1;
                    strncpy (msg, read_buf+i, size_msg_t);
                    msg[size_msg_t+1] = '\0';   /* null character manually added */
                    /*RCLCPP_INFO(this->get_logger(), "Response: %s", msg);
                    RCLCPP_INFO(this->get_logger(), "First character: %c", msg[0]);
                    RCLCPP_INFO(this->get_logger(), "Size msg: %i", (int)(sizeof msg));*/


                    DeserializationError err = deserializeJson(doc_got, msg);

                    if (err) 
                    {
                        RCLCPP_INFO(this->get_logger(), "deserializeJson() failed with code %s \n", err.c_str());
                        
                    }
                    else
                    {
                        res[0] = doc_got["data1"].as<int>();
                        res[1] = doc_got["data2"].as<int>();
                        res[2] = doc_got["data3"].as<int>();
                        res[3] = doc_got["data4"].as<int>();
                        res[4] = doc_got["data5"].as<int>();
                        res[5] = doc_got["data6"].as<int>();
                        res[6] = doc_got["data7"].as<int>();
                        res[7] = doc_got["data8"].as<int>();

                        RCLCPP_INFO(this->get_logger(), "Reception: %i, %i, %i, %i, %i, %i, %i, %i", doc_got["data1"].as<int>(), res[1], res[2], res[3], res[4], res[5], res[6], res[7]);
                    }
                    break;
                }
            }
        }
    }

    //Reception et lecture d'un JSON
    void deserialize_json()
    {
        RCLCPP_INFO(this->get_logger(), "Check1");
        
        char read_buf [max_size];
        char msg [size_msg+1];

        bool verif = 0;

        memset(read_buf, '\0', sizeof read_buf);
        // Read bytes. The behaviour of read() (e.g. does it block?,
        // how long does it block for?) depends on the configuration
        // settings above, specifically VMIN and VTIME
        int n = read(serial_port, &read_buf, sizeof(read_buf));

        RCLCPP_INFO(this->get_logger(), "Check2");
        
        if (n < 0) {
            RCLCPP_INFO(this->get_logger(), "Error reading: %s", strerror(errno));
        }
        else if (n == 0) {
            RCLCPP_INFO(this->get_logger(), "Read nothing!");
        }
        else {

            StaticJsonDocument<capacity_got> doc_got;
            for(int i=0; i<max_size; i++)
            {
                if(read_buf[i] == '{' && (i+size_msg)<max_size && verif == 0)
                {
                    verif = 1;
                    strncpy (msg, read_buf+i, size_msg);
                    msg[sizeof(msg)] = '\0';   /* null character manually added */
                    /*RCLCPP_INFO(this->get_logger(), "Response: %s", msg);
                    RCLCPP_INFO(this->get_logger(), "First character: %c", msg[0]);
                    RCLCPP_INFO(this->get_logger(), "Size msg: %i", (int)(sizeof msg));*/

                    RCLCPP_INFO(this->get_logger(), "msg: %s", msg);

                    DeserializationError err = deserializeJson(doc_got, msg);

                    if (err) 
                    {
                        RCLCPP_INFO(this->get_logger(), "deserializeJson() failed with code %s \n", err.c_str());  
                    }
                    else
                    {
                        //Imu
                        imu_data.linear_acceleration.x = doc_got["imu"]["a_x"].as<float>();
                        imu_data.linear_acceleration.y = doc_got["imu"]["a_y"].as<float>();
                        imu_data.linear_acceleration.z = doc_got["imu"]["a_z"].as<float>();

                        imu_data.angular_velocity.x = doc_got["imu"]["p"].as<float>();
                        imu_data.angular_velocity.y = doc_got["imu"]["q"].as<float>();
                        imu_data.angular_velocity.z = doc_got["imu"]["r"].as<float>();

                        /*imu_data.orientation.x = doc_got["imu"]["q1"].as<float>();
                        imu_data.orientation.y = doc_got["imu"]["q2"].as<float>();
                        imu_data.orientation.z = doc_got["imu"]["q3"].as<float>();
                        imu_data.orientation.w = doc_got["imu"]["q4"].as<float>();*/

                        magnetic_data.magnetic_field.x = doc_got["imu"]["mag_x"].as<float>();
                        magnetic_data.magnetic_field.y = doc_got["imu"]["mag_y"].as<float>();
                        magnetic_data.magnetic_field.z = doc_got["imu"]["mag_z"].as<float>();

                        angle_data.x = doc_got["imu"]["roll"].as<float>();
                        angle_data.y = doc_got["imu"]["pitch"].as<float>();
                        angle_data.z = doc_got["imu"]["yaw"].as<float>();

                        //Pression
                        press_data.fluid_pressure = doc_got["press"]["press"].as<float>();
                        depth_data.data = doc_got["press"]["depth"].as<float>();

                        //Tension
                        //ten_data.data = doc_got["ten"]["tension"].as<float>();

                        //Hygrometre
                        //hygro_data.relative_humidity = doc_got["hygro"]["hygro"].as<float>();

                        //Temperature
                        //temp_data.temperature = doc_got["temp"]["temp"].as<float>();

                        //Publication
                        imu_pub_->publish(imu_data);
                        magnetic_pub_->publish(magnetic_data);
                        angle_pub_->publish(angle_data);
                        pressure_pub_->publish(press_data);
                        depth_pub_->publish(depth_data);
                        temperature_pub_->publish(temp_data);
                        humidity_pub_->publish(hygro_data);
                        tension_pub_->publish(ten_data);

                        RCLCPP_INFO(this->get_logger(), "Imu: a_x: %f, a_y: %f, a_z: %f, p: %f, q: %f, r: %f", 
                            imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z,
                            imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z);
                        //RCLCPP_INFO(this->get_logger(), "Imu: roll: %f, pitch: %f, yaw: %f, q1: %f, q2: %f, q3: %f, q4: %f",
                        //    angle_data.x, angle_data.y, angle_data.z, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
                        RCLCPP_INFO(this->get_logger(), "Imu: mag_x: %f, mag_y: %f, mag_z: %f",
                            magnetic_data.magnetic_field.x, magnetic_data.magnetic_field.y, magnetic_data.magnetic_field.z);
                        //RCLCPP_INFO(this->get_logger(), "press: %f, depth: %f, tension: %f, humidite: %f, temperature: %f", 
                        //    press_data.fluid_pressure , depth_data.data, ten_data.data , hygro_data.relative_humidity, temp_data.temperature);
                    }    
                    break;
                }
            }
        }

    }

    void deserialize_json2()
    {
        RCLCPP_INFO(this->get_logger(), "Check1");
        
        char read_buf [max_size];
        char msg [size_msg+1];

        bool verif = 0;

        memset(read_buf, '\0', sizeof read_buf);

        RCLCPP_INFO(this->get_logger(), "Check2");
        
        //Attendre d'avoir le premier caractere du message (accolade car le message est un JSON) tout en ne bloquant pas indefinimement le programme
        //si le message n'arrive pas : duree max 100ms
        // Get simulator time
        start_time = rclcpp::Clock{RCL_ROS_TIME}.now();
        dt = 0.0;
        
        while(dt < 0.2)
        {
            int n = read(serial_port, &read_buf, sizeof(read_buf));

            if (n < 0) {
                RCLCPP_INFO(this->get_logger(), "Error reading: %s", strerror(errno));
            }
            else if (n == 0) {
                RCLCPP_INFO(this->get_logger(), "Read nothing!");
            }
            else 
            {
                RCLCPP_INFO(this->get_logger(), "buffer: %s", read_buf);
                StaticJsonDocument<capacity_got> doc_got;
                for(int i=0; i<max_size; i++)
                {
                    if(read_buf[i] == '{' && read_buf[i+1] == '"' && read_buf[i+2] == 'i' && read_buf[i+2] == 'm' && (i+size_msg)<max_size && verif == 0)
                    //if(read_buf[i] == '\n' && (i+size_msg)<max_size && verif == 0)
                    {
                        verif = 1;
                        strncpy (msg, read_buf+i, size_msg);
                        msg[sizeof(msg)] = '\0';   /* null character manually added */
                        /*RCLCPP_INFO(this->get_logger(), "Response: %s", msg);
                        RCLCPP_INFO(this->get_logger(), "First character: %c", msg[0]);
                        RCLCPP_INFO(this->get_logger(), "Size msg: %i", (int)(sizeof msg));*/

                        RCLCPP_INFO(this->get_logger(), "msg: %s", msg);

                        DeserializationError err = deserializeJson(doc_got, msg);

                        if (err) 
                        {
                            RCLCPP_INFO(this->get_logger(), "deserializeJson() failed with code %s \n", err.c_str());  
                        }
                        else
                        {
                            //Imu
                            imu_data.linear_acceleration.x = doc_got["imu"]["a_x"].as<float>();
                            imu_data.linear_acceleration.y = doc_got["imu"]["a_y"].as<float>();
                            imu_data.linear_acceleration.z = doc_got["imu"]["a_z"].as<float>();

                            imu_data.angular_velocity.x = doc_got["imu"]["p"].as<float>();
                            imu_data.angular_velocity.y = doc_got["imu"]["q"].as<float>();
                            imu_data.angular_velocity.z = doc_got["imu"]["r"].as<float>();

                            /*imu_data.orientation.x = doc_got["imu"]["q1"].as<float>();
                            imu_data.orientation.y = doc_got["imu"]["q2"].as<float>();
                            imu_data.orientation.z = doc_got["imu"]["q3"].as<float>();
                            imu_data.orientation.w = doc_got["imu"]["q4"].as<float>();*/

                            magnetic_data.magnetic_field.x = doc_got["imu"]["mag_x"].as<float>();
                            magnetic_data.magnetic_field.y = doc_got["imu"]["mag_y"].as<float>();
                            magnetic_data.magnetic_field.z = doc_got["imu"]["mag_z"].as<float>();

                            angle_data.x = doc_got["imu"]["roll"].as<float>();
                            angle_data.y = doc_got["imu"]["pitch"].as<float>();
                            angle_data.z = doc_got["imu"]["yaw"].as<float>();

                            //Pression
                            press_data.fluid_pressure = doc_got["press"]["press"].as<float>();
                            depth_data.data = doc_got["press"]["depth"].as<float>();

                            //Tension
                            //ten_data.data = doc_got["ten"]["tension"].as<float>();

                            //Hygrometre
                            //hygro_data.relative_humidity = doc_got["hygro"]["hygro"].as<float>();

                            //Temperature
                            //temp_data.temperature = doc_got["temp"]["temp"].as<float>();

                            //Publication
                            imu_pub_->publish(imu_data);
                            magnetic_pub_->publish(magnetic_data);
                            angle_pub_->publish(angle_data);
                            pressure_pub_->publish(press_data);
                            depth_pub_->publish(depth_data);
                            temperature_pub_->publish(temp_data);
                            humidity_pub_->publish(hygro_data);
                            tension_pub_->publish(ten_data);

                            RCLCPP_INFO(this->get_logger(), "Imu: a_x: %f, a_y: %f, a_z: %f, p: %f, q: %f, r: %f", 
                                imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z,
                                imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z);
                            //RCLCPP_INFO(this->get_logger(), "Imu: roll: %f, pitch: %f, yaw: %f, q1: %f, q2: %f, q3: %f, q4: %f",
                            //    angle_data.x, angle_data.y, angle_data.z, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
                            RCLCPP_INFO(this->get_logger(), "Imu: mag_x: %f, mag_y: %f, mag_z: %f",
                                magnetic_data.magnetic_field.x, magnetic_data.magnetic_field.y, magnetic_data.magnetic_field.z);
                            //RCLCPP_INFO(this->get_logger(), "press: %f, depth: %f, tension: %f, humidite: %f, temperature: %f", 
                            //    press_data.fluid_pressure , depth_data.data, ten_data.data , hygro_data.relative_humidity, temp_data.temperature);
                        }    
                        break;
                    }
                }
            }

            current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
            dt = current_time.seconds() - start_time.seconds();
        }

    }

    void deserialize_json3()
    {
        RCLCPP_INFO(this->get_logger(), "Check1");
        
        char read_buf [max_size];
        char msg [size_msg+1];

        bool verif = 0;

        memset(read_buf, '\0', sizeof read_buf);

        RCLCPP_INFO(this->get_logger(), "Check2");
        
        //Attendre d'avoir le premier caractere du message (accolade car le message est un JSON) tout en ne bloquant pas indefinimement le programme
        //si le message n'arrive pas : duree max 100ms
        // Get simulator time
        start_time = rclcpp::Clock{RCL_ROS_TIME}.now();
        dt = 0.0;
        
        while(dt < 0.01)
        {
            int n = read(serial_port, &read_buf, sizeof(read_buf));

            if (n < 0) {
                int a = 1;
                //RCLCPP_INFO(this->get_logger(), "Error reading: %s", strerror(errno));
            }
            else if (n == 0) {
                RCLCPP_INFO(this->get_logger(), "Read nothing!");
            }
            else 
            {
                RCLCPP_INFO(this->get_logger(), "buffer: %s", read_buf);
                StaticJsonDocument<capacity_got> doc_got;
                for(int i=0; i<max_size; i++)
                {
                    //RCLCPP_INFO(this->get_logger(), "character: %c, i+size_msg : %i, verif : %i ", read_buf[i], i+size_msg, verif);
                    if(read_buf[i] == '{' && (i+size_msg)<max_size)
                    //if(read_buf[i] == '\n' && (i+size_msg)<max_size && verif == 0)
                    {
                        int max;
                        for(int j=i; j<max_size; j++)
                        {
                            if(read_buf[j] == '}')
                            {
                                max = j;
                                break;
                            }
                        }
                        
                        verif = 1;
                        strncpy (msg, read_buf+i, max+1);
                        //msg[sizeof(msg)] = '\0';   /* null character manually added */
                        msg[max+1] = '\0'; 
                        /*RCLCPP_INFO(this->get_logger(), "Response: %s", msg);
                        RCLCPP_INFO(this->get_logger(), "First character: %c", msg[0]);
                        RCLCPP_INFO(this->get_logger(), "Size msg: %i", (int)(sizeof msg));*/

                        RCLCPP_INFO(this->get_logger(), "msg: %s", msg);

                        DeserializationError err = deserializeJson(doc_got, msg);

                        if (err) 
                        {
                            RCLCPP_INFO(this->get_logger(), "deserializeJson() failed with code %s \n", err.c_str());  
                        }
                        else
                        {
                            JsonObject obj = doc_got.as<JsonObject>();
                            
                            for (JsonPair p : obj)
                            {
                                //RCLCPP_INFO(this->get_logger(), "key obtenue: %c", p.key());
                                if(p.key() == "imu_a_x")
                                    imu_data.linear_acceleration.x = p.value().as<float>();
                                if(p.key() == "imu_a_y")
                                    imu_data.linear_acceleration.y = p.value().as<float>();
                                if(p.key() == "imu_a_z")
                                    imu_data.linear_acceleration.z = p.value().as<float>();
                                if(p.key() == "imu_p")
                                    imu_data.angular_velocity.x = p.value().as<float>();
                                if(p.key() == "imu_q")
                                    imu_data.angular_velocity.y = p.value().as<float>();
                                if(p.key() == "imu_r")
                                    imu_data.angular_velocity.z = p.value().as<float>();
                                if(p.key() == "imu_mag_x")
                                    magnetic_data.magnetic_field.x = p.value().as<float>();
                                if(p.key() == "imu_mag_y")
                                    magnetic_data.magnetic_field.y = p.value().as<float>();
                                if(p.key() == "imu_mag_z")
                                    magnetic_data.magnetic_field.z = p.value().as<float>();
                                if(p.key() == "imu_roll")
                                    angle_data.x = p.value().as<float>();
                                if(p.key() == "imu_pitch")
                                    angle_data.y = p.value().as<float>();
                                if(p.key() == "imu_yaw")
                                    angle_data.z = p.value().as<float>();
                                if(p.key() == "press_press")
                                    press_data.fluid_pressure = p.value().as<float>();
                                if(p.key() == "press_depth")
                                    depth_data.data = p.value().as<float>();

                            }

                            //Publication
                            imu_pub_->publish(imu_data);
                            magnetic_pub_->publish(magnetic_data);
                            angle_pub_->publish(angle_data);
                            pressure_pub_->publish(press_data);
                            depth_pub_->publish(depth_data);
                            temperature_pub_->publish(temp_data);
                            humidity_pub_->publish(hygro_data);
                            tension_pub_->publish(ten_data);

                            //RCLCPP_INFO(this->get_logger(), "Imu: a_x: %f, a_y: %f, a_z: %f, p: %f, q: %f, r: %f", 
                            //    imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z,
                            //    imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z);
                            //RCLCPP_INFO(this->get_logger(), "Imu: roll: %f, pitch: %f, yaw: %f, q1: %f, q2: %f, q3: %f, q4: %f",
                            //    angle_data.x, angle_data.y, angle_data.z, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
                            //RCLCPP_INFO(this->get_logger(), "Imu: mag_x: %f, mag_y: %f, mag_z: %f",
                            //    magnetic_data.magnetic_field.x, magnetic_data.magnetic_field.y, magnetic_data.magnetic_field.z);
                            //RCLCPP_INFO(this->get_logger(), "press: %f, depth: %f, tension: %f, humidite: %f, temperature: %f", 
                            //    press_data.fluid_pressure , depth_data.data, ten_data.data , hygro_data.relative_humidity, temp_data.temperature);
                            RCLCPP_INFO(this->get_logger(), "press: %f, depth: %f", 
                                press_data.fluid_pressure , depth_data.data);
                        }    
                        //break;
                    }
                }
            }

            current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
            dt = current_time.seconds() - start_time.seconds();
        }

    }

    void timer_callback()
    {
        //RCLCPP_INFO(this->get_logger(), "Boucle");
        serialize_json();
        //deserialize_json_trash();
        //deserialize_json();
        //deserialize_json2();
        deserialize_json3();
    }

  private:

    void ControlCallback(const trajectory_msgs::msg::JointTrajectoryPoint& traj_msg)
    {
        cmd_motors = traj_msg;
        //RCLCPP_INFO(this->get_logger(), "Reception comm: %lf %lf %lf %lf ", cmd_motors.velocities[0], 
        //    cmd_motors.velocities[1], cmd_motors.velocities[2], cmd_motors.velocities[3]);
    }
    
    //Subscribers
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr cmd_motors_sub_;

    //Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
    rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr humidity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tension_pub_;

    //Messages
    trajectory_msgs::msg::JointTrajectoryPoint cmd_motors;
    sensor_msgs::msg::Imu imu_data;
    sensor_msgs::msg::MagneticField magnetic_data;
    geometry_msgs::msg::Vector3 angle_data;
    sensor_msgs::msg::FluidPressure press_data;
    std_msgs::msg::Float64 depth_data;
    sensor_msgs::msg::Temperature temp_data;
    sensor_msgs::msg::RelativeHumidity hygro_data;
    std_msgs::msg::Float64 ten_data;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;

    //Time
    rclcpp::Time start_time;
    rclcpp::Time current_time;
    double dt;

    //Variables
    unsigned int baud_rate;
    static constexpr const char* port_name = "/dev/ttyACM0";
    
    int serial_port;
    // Create new termios struct, we call it 'tty' for convention
    // No need for "= {0}" at the end as we'll immediately write the existing
    // config to this struct
    struct termios tty;
    struct termios tty_old;

    static constexpr const int capacity_sent = 4 * JSON_OBJECT_SIZE(1) + JSON_ARRAY_SIZE(1) + 8 * JSON_OBJECT_SIZE(2);
    //StaticJsonDocument<capacity_sent> doc_sent;

    //static constexpr const int capacity_got = 10 * JSON_ARRAY_SIZE(1) + 15 * JSON_OBJECT_SIZE(2);
    static constexpr const int capacity_got = 4 * JSON_OBJECT_SIZE(1);
    //StaticJsonDocument<capacity_got> doc_got;

    int max_size = 200;
    int size_msg = 100;

    //Test trash
    int res[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int max_size_t = 500;
    int size_msg_t = 81;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Serial_com>());
  rclcpp::shutdown();
  return 0;
}
