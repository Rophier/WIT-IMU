//#include "ros/ros.h"
#include <chrono>
#include <serial/serial.h> //ROS已经内置了的串口包
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"

#include "JY901.h"

using namespace std::chrono_literals;

class WitImuNode : public rclcpp::Node
{

public:
    WitImuNode()
    : Node("wit_imu_node")
    {
        this->configure();
        pub_imu = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 1000);
        pub_mag = this->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic, 1000);
        tf_broadcaster =std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        imu = CJY901();
        try
        {
            ser.setPort(port);
            ser.setBaudrate(baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(500);
            ser.setTimeout(to);
            ser.open();
        }
        catch (serial::IOException &e)
        {
            RCLCPP_INFO(this->get_logger(), "Unable to open port ");
            return;
        }
        if (ser.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "Serial Port initialized");
        }
        else
        {
            return;
        }
        // ser.flush();
        int size;
        timer_ = this->create_wall_timer(
            timer_ms, std::bind(&WitImuNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        int count = ser.available();
        if (count != 0)
        {
            //ROS_INFO_ONCE("Data received from serial port.");
            int num;
            rclcpp::Time now = this->get_clock()->now();
            unsigned char read_buf[count];
            num = ser.read(read_buf, count);
            imu.FetchData((char *)read_buf, num);
            sensor_msgs::msg::Imu imu_data;

            imu_data.header.stamp = now;
            imu_data.header.frame_id = imu_frame;

            imu_data.linear_acceleration.x = imu.acc.x;
            imu_data.linear_acceleration.y = imu.acc.y;
            imu_data.linear_acceleration.z = imu.acc.z;
            imu_data.linear_acceleration_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};
            //RCLCPP_INFO(this->get_logger(), "Accel: x=%f, y=%f, z=%f",
            //   imu.acc.x, imu.acc.y, imu.acc.z);
            imu_data.angular_velocity.x = imu.gyro.x;
            imu_data.angular_velocity.y = imu.gyro.y;
            imu_data.angular_velocity.z = imu.gyro.z;
            imu_data.linear_acceleration_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};

            imu_data.orientation.x = imu.quat.x;
            imu_data.orientation.y = imu.quat.y;
            imu_data.orientation.z = imu.quat.z;
            imu_data.orientation.w = imu.quat.w;
            imu_data.orientation_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};
            //RCLCPP_INFO(this->get_logger(), "Quaternion: x=%f, y=%f, z=%f, w=%f", 
            //   imu.quat.x, imu.quat.y, imu.quat.z, imu.quat.w);
            pub_imu->publish(imu_data);
            if(is_pub_tf) {
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = now;
                t.header.frame_id = this->base_frame;
                t.child_frame_id = this->imu_frame;
                t.transform.rotation.x = imu.quat.x;
                t.transform.rotation.y = imu.quat.y;
                t.transform.rotation.z = imu.quat.z;
                t.transform.rotation.w = imu.quat.w;
                this->tf_broadcaster->sendTransform(t);
            }

            if (is_pub_mag)
            {
                sensor_msgs::msg::MagneticField mag_data;

                mag_data.header.stamp = imu_data.header.stamp;
                mag_data.header.frame_id = imu_data.header.frame_id;

                mag_data.magnetic_field.x = imu.mag.x;
                mag_data.magnetic_field.y = imu.mag.y;
                mag_data.magnetic_field.z = imu.mag.z;
                mag_data.magnetic_field_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};

                pub_mag->publish(mag_data);
            }
        }
    }
    void configure()
    {
        this->declare_parameter<std::string>("port.name",       "/dev/ttyUSB0");
        this->declare_parameter<int>        ("port.baudrate",   9600);
        this->declare_parameter<std::string>("imu.topic",       "imu_data");
        this->declare_parameter<int>        ("imu.output_hz",   20);
        this->declare_parameter<bool>       ("mag.enable",      false);
        this->declare_parameter<std::string>("mag.topic",       "mag_data");
        this->declare_parameter<bool>       ("tf.enable",       false);
        this->declare_parameter<std::string>("tf.base_frame",   "base_link");
        this->declare_parameter<std::string>("tf.imu_frame",    "imu_link");

        this->get_parameter<std::string>    ("port.name",       port);
        this->get_parameter<int>            ("port.baudrate",   baudrate);
        this->get_parameter<int>            ("imu.output_hz",   output_hz);
        this->get_parameter<std::string>    ("imu.topic",       imu_topic);
        this->get_parameter<bool>           ("mag.enable",      is_pub_mag);
        this->get_parameter<std::string>    ("mag.topic",       mag_topic);
        this->get_parameter<bool>           ("tf.enable",       is_pub_tf);
        this->get_parameter<std::string>    ("tf.base_frame",   base_frame);
        this->get_parameter<std::string>    ("tf.imu_frame",    imu_frame);
        RCLCPP_INFO(this->get_logger(), "Params:");
        RCLCPP_INFO(this->get_logger(), "\tport.Name: %s",      port.c_str());
        RCLCPP_INFO(this->get_logger(), "\tport.Baudrate: %d",  baudrate);
        RCLCPP_INFO(this->get_logger(), "\tIMU.Ouput_Hz: %d",   output_hz);
        timer_ms = std::chrono::milliseconds {static_cast<long int>(1000 / output_hz)};
        RCLCPP_INFO(this->get_logger(), "\tMilliseconds: %d",   timer_ms.count());
        RCLCPP_INFO(this->get_logger(), "\tIMU.Topic: %s",      imu_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "\tMAG.Enable: %d",     is_pub_mag);
        RCLCPP_INFO(this->get_logger(), "\tMAG.Topic: %s",      mag_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "\tTF.Base_link: %s",   base_frame.c_str());
        RCLCPP_INFO(this->get_logger(), "\tTF.Imu_link: %s",    imu_frame.c_str());

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr              pub_imu;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr    pub_mag;
    CJY901 imu;
    std::string port;
    int baudrate;
    int pub_rate;
    int output_hz;
    bool is_pub_mag;
    serial::Serial ser;
    bool is_pub_tf;
    std::chrono::milliseconds timer_ms;
    std::string imu_topic;
    std::string mag_topic;
    std::string imu_frame;
    std::string base_frame;
    std::unique_ptr<tf2_ros::TransformBroadcaster>              tf_broadcaster;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WitImuNode>());
    rclcpp::shutdown();
    return 0;
}