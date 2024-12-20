// Saves a pointcloud topic to a file
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

#include "pcd_lib.h"

using std::string;

class PCDSubscriber : public rclcpp::Node
{
public: PCDSubscriber() : Node("pcd_subsriber"){
    // TODO: save the pointcloud with a frame_id to another frame_id with transformation callback lookupTransform
    this->declare_parameter<std::string>("pcd_file_path", pcd_file_path_);
    this->declare_parameter<std::string>("topic_name", "pointcloud");
    this->declare_parameter<std::string>("frame_id", "none");
    // TODO: add a parameter for continous saving
    this->declare_parameter<bool>("continuous_saving", false);
    this->declare_parameter<double>("continuous_saving_rate", 1.0);
    this->get_parameter("pcd_file_path", pcd_file_path_);
    this->get_parameter("topic_name", topic_name_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("continuous_saving", continuous_saving_);
    this->get_parameter("continuous_saving_rate", continuous_saving_rate_);


    RCLCPP_INFO(this->get_logger(), "Topic name: %s", topic_name_.c_str());
    this->dummy = true;

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name_, 10, std::bind(&PCDSubscriber::callback, this, std::placeholders::_1));
}
private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (this->dummy) {
            pcd_save_to_ascii_file(this->get_logger(), msg, "output.pcd");
            this->dummy = false;
        }
        return;

        RCLCPP_INFO_STREAM(this->get_logger(), "Received pointcloud message with " << msg->width * msg->height << " points, topic: " << topic_name_);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        string pointcloud_frame_id_ = msg->header.frame_id;
        geometry_msgs::msg::TransformStamped transformStamped, transformInverse;
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        if(frame_id_ != "none"){
            try
            {
                transformStamped = tf_buffer_->lookupTransform(pointcloud_frame_id_, frame_id_, tf2::TimePointZero);
                transformInverse = tf_buffer_->lookupTransform(frame_id_, pointcloud_frame_id_, tf2::TimePointZero);
            }

            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
                // return;
            }
            // TODO: transform the pointcloud to save
        }
        if (pcl::io::savePCDFileASCII(pcd_file_path_, *cloud) == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Saved %d data points to %s", cloud->width * cloud->height, pcd_file_path_.c_str());
            // shutdown node
            rclcpp::shutdown();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save data points to %s", pcd_file_path_.c_str());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    string pcd_file_path_;
    string topic_name_;
    string frame_id_;
    bool continuous_saving_;
    bool dummy;
    double continuous_saving_rate_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCDSubscriber>());
    rclcpp::shutdown();
    return 0;
}
