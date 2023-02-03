#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers;
        this->declare_parameter("kp", 6.1);
        this->declare_parameter("kd", 0.021);
        this->declare_parameter("ki", 0.0);
        this->declare_parameter("theta", 45.0*M_PI/180.0);
        this->declare_parameter("L", 1.0);
        this->declare_parameter("dist_to_wall", 1.2);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));\
        
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);
    }

private:
    // PID CONTROL PARAMS

    double prev_time;

    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;

    double angle_min, angle_increment;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    double get_range(const float* range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        int pos = (angle - angle_min) / angle_increment;
        if(range_data[pos] == std::numeric_limits<float>::infinity() || std::isnan(range_data[pos]))
        {
            return 0.0;
        }
        return *(range_data + pos);
    }

    double get_error(const float* range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        double b = get_range(range_data, 90.0*M_PI/180.0);
        double a = get_range(range_data, this->get_parameter("theta").get_parameter_value().get<double>());

        if(b == 0.0 || a == 0.0){
            return 0.0;
        }

        double alpha = atan2(a*cos(this->get_parameter("theta").get_parameter_value().get<double>()) - b, a*sin(this->get_parameter("theta").get_parameter_value().get<double>()));
        double dt = b*cos(alpha);

        // RCLCPP_INFO(this->get_logger(), "alpha:%f, b:%f, a:%f", alpha, b, a);

        double dtp1 = dt + this->get_parameter("L").get_parameter_value().get<double>()*sin(alpha);
        // RCLCPP_INFO(this->get_logger(), "dt: %f", dt);
        return dtp1 - dist;
    }

    void pid_control(double error, double time)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
       
        // TODO: Use kp, ki & kd to implement a PID controller
        double delta_t = time - prev_time;
        integral += error * delta_t;

        double steering_angle = this->get_parameter("kp").get_parameter_value().get<double>() * error + 
                            this->get_parameter("kd").get_parameter_value().get<double>() * (prev_error - error) / delta_t + 
                            this->get_parameter("ki").get_parameter_value().get<double>() * integral;

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
        drive_msg.drive.steering_angle = steering_angle;
        if (steering_angle < 10*M_PI/180 && steering_angle > -10*M_PI/180)
            drive_msg.drive.speed = 1.5;
        else if (steering_angle < 20*M_PI/180 && steering_angle > -20*M_PI/180)
            drive_msg.drive.speed = 1.0;
        else
            drive_msg.drive.speed = 0.5;

        drive_pub_->publish(drive_msg);

        prev_time = time;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */

        angle_min = scan_msg->angle_min;
        angle_increment = scan_msg->angle_increment;

        const float *range_data = scan_msg->ranges.data();
        error = get_error(range_data, this->get_parameter("dist_to_wall").get_parameter_value().get<double>());
        pid_control(error, scan_msg->header.stamp.sec + scan_msg->header.stamp.nanosec * 1e-9);
        
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}