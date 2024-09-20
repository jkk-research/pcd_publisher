#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

class PCDPublisher : public rclcpp::Node
{
public:
    PCDPublisher(const std::string& pcd_file_path)
    : Node("pcd_publisher"), pcd_file_path_(pcd_file_path)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PCDPublisher::publish_pointcloud, this));

        // Load PCD file
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, *cloud_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", pcd_file_path_.c_str());
            rclcpp::shutdown();
        }
    }

private:
    void publish_pointcloud()
    {
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_, output);
        output.header.frame_id = "map";
        output.header.stamp = this->now();
        publisher_->publish(output);
        RCLCPP_INFO(this->get_logger(), "Published PointCloud2 message");
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string pcd_file_path_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_{new pcl::PointCloud<pcl::PointXYZ>};
};

int main(int argc, char* argv[])
{

    auto pcd_path = ament_index_cpp::get_package_share_directory("pcd_publisher") + "/maps/" + "map.pcd";

    // std::string pcd_path = "";
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCDPublisher>(pcd_path));
    rclcpp::shutdown();
    return 0;
}