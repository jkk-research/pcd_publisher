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
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

#include "pcd_lib.h"

using std::string;

class PCDSubscriber : public rclcpp::Node
{
public: PCDSubscriber() : Node("pcd_subsriber") {
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

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);



    RCLCPP_INFO(this->get_logger(), "Topic name: %s", topic_name_.c_str());
    this->dummy = true;

    //tf_buffer_.setUsingDedicatedThread(true);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name_, 10, std::bind(&PCDSubscriber::callback, this, std::placeholders::_1));
}
private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
#if 0
        if (this->dummy) {
            //pcd_save_to_ascii_file(this->get_logger(), msg, "output.pcd");
            pcd_save_to_binary_file(this->get_logger(), msg, "output.pcd");
            this->dummy = false;
        }
        return;
#endif

        RCLCPP_INFO_STREAM(this->get_logger(), "Received pointcloud message with " << msg->width * msg->height << " points, topic: " << topic_name_);

        string source_frame_id = msg->header.frame_id;
        string target_frame_id = "map";

        sensor_msgs::msg::PointCloud2 transformed_cloud;
        sensor_msgs::msg::PointCloud2::SharedPtr transformed_cloud_ptr;

        RCLCPP_INFO(this->get_logger(), "source (%s) target (%s)", source_frame_id.c_str(), target_frame_id.c_str());

        if (source_frame_id != "none") {
            try {
                if (tf_buffer_->canTransform(target_frame_id, source_frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0))) {
                    geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                        target_frame_id,
                        source_frame_id,
                        tf2::TimePointZero
                    );

                    RCLCPP_INFO(this->get_logger(), "transformation (x=%f, y=%f, z=%f)",
                        transform_stamped.transform.translation.x,
                        transform_stamped.transform.translation.y,
                        transform_stamped.transform.translation.z
                    );

                    tf2::doTransform(*msg, transformed_cloud, transform_stamped);
                    transformed_cloud.header.frame_id = target_frame_id;
                    transformed_cloud_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(transformed_cloud);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Transform from %s to %s is not available yet", source_frame_id.c_str(), target_frame_id.c_str());
                }


            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Failed to transform PointCloud2: %s", ex.what());
            }
        } else {
            transformed_cloud_ptr = msg;
        }

        // Save the transformed PointCloud
#if 0
        if (!pcd_save_to_binary_file(this->get_logger(), transformed_cloud_ptr, "output.pcd")) {
            // shutdown node
            rclcpp::shutdown();
        }
#endif
        return;

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
