#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>


using std::string;

class PCDPublisher : public rclcpp::Node
{
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &param : parameters)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
            if (param.get_name() == "pcd_file_path")
            {
                pcd_file_path_ = param.as_string();
            }
            else if (param.get_name() == "frame_id")
            {
                frame_id_ = param.as_string();
            }
            else if (param.get_name() == "x_translation")
            {
                x_translation_ = param.as_double();
            }
            else if (param.get_name() == "y_translation")
            {
                y_translation_ = param.as_double();
            }
            else if (param.get_name() == "z_translation")
            {
                z_translation_ = param.as_double();
            }
            else if (param.get_name() == "x_rotation")
            {
                x_rotation_ = param.as_double();
            }
            else if (param.get_name() == "y_rotation")
            {
                y_rotation_ = param.as_double();
            }
            else if (param.get_name() == "z_rotation")
            {
                z_rotation_ = param.as_double();
            }
            else if (param.get_name() == "field_to_publish") // New parameter
            {
                field_to_publish_ = param.as_string();
            }
            else
            {
                result.successful = false;
                result.reason = "parameter not found";
            }

        }
        return result;
    }

public:
    PCDPublisher()
    : Node("pcd_publisher")
    {
        this->declare_parameter<std::string>("pcd_file_path", pcd_file_path_);
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<std::string>("topic_name", "pointcloud");
        this->declare_parameter<double>("x_translation", x_translation_);
        this->declare_parameter<double>("y_translation", y_translation_);
        this->declare_parameter<double>("z_translation", z_translation_);
        this->declare_parameter<double>("x_rotation", x_rotation_);
        this->declare_parameter<double>("y_rotation", y_rotation_);
        this->declare_parameter<double>("z_rotation", z_rotation_);
        this->declare_parameter<std::string>("field_to_publish", "intensity"); // New parameter
        this->get_parameter("pcd_file_path", pcd_file_path_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("topic_name", topic_name_);
        this->get_parameter("x_translation", x_translation_);
        this->get_parameter("y_translation", y_translation_);
        this->get_parameter("z_translation", z_translation_);
        this->get_parameter("x_rotation", x_rotation_);
        this->get_parameter("y_rotation", y_rotation_);
        this->get_parameter("z_rotation", z_rotation_);
        this->get_parameter("field_to_publish", field_to_publish_); // Get the new parameter
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_, 10); 
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PCDPublisher::parametersCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PCDPublisher::publish_pointcloud, this));

        RCLCPP_INFO_STREAM(this->get_logger(), "PCD file path:  " << pcd_file_path_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Frame ID:       " << frame_id_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Topic name:     " << topic_name_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Translation:    " << x_translation_ << ", " << y_translation_ << ", " << z_translation_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Rotation (rad): " << x_rotation_ << ", " << y_rotation_ << ", " << z_rotation_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Rotation (deg): " << x_rotation_ * 180.0 / M_PI << ", " << y_rotation_ * 180.0 / M_PI << ", " << z_rotation_ * 180.0 / M_PI);
        RCLCPP_INFO_STREAM(this->get_logger(), "Field to publish: " << field_to_publish_); // Log the new parameter

        // Load PCD file
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file_path_, *cloud_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", pcd_file_path_.c_str());
            rclcpp::shutdown();
        }
    }

private:
    void publish_pointcloud()
    {
        // Apply translation and rotation to the point cloud
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << x_translation_, y_translation_, z_translation_;
        transform.rotate(Eigen::AngleAxisf(x_rotation_, Eigen::Vector3f::UnitX()));
        transform.rotate(Eigen::AngleAxisf(y_rotation_, Eigen::Vector3f::UnitY()));
        transform.rotate(Eigen::AngleAxisf(z_rotation_, Eigen::Vector3f::UnitZ()));
        pcl::transformPointCloud(*cloud_, *cloud_transformed_, transform);

        // Ensure the PointCloud2 message contains the selected field (rgb or intensity)
        bool has_field = false;
        for (const auto& point : cloud_transformed_->points) {
            if (field_to_publish_ == "rgb" || field_to_publish_ == "intensity") {
                has_field = true;
                break;
            }
        }

        if (!has_field) {
            RCLCPP_WARN(this->get_logger(), "PointCloud2 message does not contain the '%s' field. Data not published!", field_to_publish_.c_str());
            return;
        }

        // Create ROS 2 PointCloud2 message
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_transformed_, output);
        output.header.frame_id = frame_id_;
        output.header.stamp = this->now();
        publisher_->publish(output);
        RCLCPP_INFO_ONCE(this->get_logger(), "Published PointCloud2 message");
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    string pcd_file_path_, frame_id_, topic_name_, field_to_publish_; // New member variable
    double x_translation_{0.0}, y_translation_{0.0}, z_translation_{0.0};
    double x_rotation_{0.0}, y_rotation_{0.0}, z_rotation_{0.0};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed_{new pcl::PointCloud<pcl::PointXYZRGB>};
};

int main(int argc, char* argv[])
{
    // if (argc != 2)
    // {
    //     std::cerr << "Usage: " << argv[0] << " <pcd_file_path>" << std::endl;
    //     return 1;
    // }

    // uncomment this if you wanna use a file inside of the package's share directory
    // string pcd_file = "map.pcd"; 
    // auto pcd_path = ament_index_cpp::get_package_share_directory("pcd_publisher") + "/maps/" + pcd_file;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCDPublisher>());
    rclcpp::shutdown();
    return 0;
}