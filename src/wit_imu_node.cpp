//#include "ros/ros.h"
#include <chrono>
#include <serial/serial.h> //ROS已经内置了的串口包
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
// #include <std_msgs/Empty.h>

#include "rclcpp/rclcpp.hpp"

#include "JY901.h"

using namespace std::chrono_literals;

class WitImuNode : public rclcpp::Node
{

public:
    WitImuNode()
    : Node("wit_imu_node")
    {
        this->port = "/dev/ttyUSB0";
        this->imu_frame = "imu_link";
        pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 1000);
        pub_mag = this->create_publisher<sensor_msgs::msg::MagneticField>("mag_data", 1000);
        imu = CJY901();
        try
        {
            //设置串口属性，并打开串口
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
        //检测串口是否已经打开，并给出提示信息
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
            50ms, std::bind(&WitImuNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        int count = ser.available();
        if (count != 0)
        {
            //ROS_INFO_ONCE("Data received from serial port.");
            int num;
            unsigned char read_buf[count];
            num = ser.read(read_buf, count);
            imu.FetchData((char *)read_buf, num);
            // ROS_INFO("IMU Data : Quaternion{ x: %f  y: %f  z: %f  w: %f} ", imu.quat.x, imu.quat.y, imu.quat.z, imu.quat.w);
            sensor_msgs::msg::Imu imu_data;

            imu_data.header.stamp = this->get_clock()->now();
            imu_data.header.frame_id = imu_frame;

            imu_data.linear_acceleration.x = imu.acc.x;
            imu_data.linear_acceleration.y = imu.acc.y;
            imu_data.linear_acceleration.z = imu.acc.z;
            imu_data.linear_acceleration_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};

            imu_data.angular_velocity.x = imu.gyro.x;
            imu_data.angular_velocity.y = imu.gyro.y;
            imu_data.angular_velocity.z = imu.gyro.z;
            imu_data.linear_acceleration_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};

            imu_data.orientation.x = imu.quat.x;
            imu_data.orientation.y = imu.quat.y;
            imu_data.orientation.z = imu.quat.z;
            imu_data.orientation.w = imu.quat.w;
            imu_data.orientation_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};

            pub_imu->publish(imu_data);

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
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag;
    CJY901 imu;
    std::string port;
    int baudrate;
    int pub_rate;
    bool is_pub_mag;
    serial::Serial ser; //声明串口对象
    std::string imu_frame;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WitImuNode>());
    rclcpp::shutdown();
    return 0;
}