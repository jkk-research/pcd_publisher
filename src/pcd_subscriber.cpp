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
#include <mutex>

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
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<bool>("continuous_saving", true);
    this->declare_parameter<double>("continuous_saving_rate", 1.0);
    this->declare_parameter<int>("pointcloud_buffer_size", 128); // buffer size in MB
    this->get_parameter("pcd_file_path", pcd_file_path_);
    this->get_parameter("topic_name", topic_name_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("continuous_saving", continuous_saving_);
    this->get_parameter("continuous_saving_rate", continuous_saving_rate_);
    this->get_parameter("pointcloud_buffer_size", pointcloud_buffer_size_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(20.0));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize merged cloud
    merged_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    merged_cloud_->header.frame_id = "map";
    merged_cloud_->height = 1;
    merged_cloud_->width = 0;
    merged_cloud_->is_dense = true;
    merged_cloud_->data.reserve(MB(pointcloud_buffer_size_));


    RCLCPP_INFO(this->get_logger(), "Topic name: %s", topic_name_.c_str());
    this->dummy = true;

    //tf_buffer_.setUsingDedicatedThread(true);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name_, 10, std::bind(&PCDSubscriber::callback, this, std::placeholders::_1));

#if 0
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / continuous_saving_rate_),
        std::bind(&PCDSubscriber::savePointCloud, this)
    );
#endif

}

    ~PCDSubscriber() {
        RCLCPP_INFO(this->get_logger(), "Saving the remaining data...");

		if (!pcd_save_to_binary_file(this->get_logger(), merged_cloud_, "final.pcd")) {
			RCLCPP_ERROR(this->get_logger(), "Couldn't save PCD file!");
			rclcpp::shutdown();
			return;
		}
    }

private:
    void appendPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Hozzáfűzöm az új pointcloudot!");

		if (merged_cloud_->width > 0 &&
           (merged_cloud_->fields != msg->fields || merged_cloud_->point_step != msg->point_step))
        {
            RCLCPP_ERROR(this->get_logger(), "A PointCloud2 struktúrák nem kompatibilisek! Adatok eldobva.");
            return;
        }

		if (merged_cloud_->width == 0)
        {
            merged_cloud_->fields = msg->fields;
            merged_cloud_->point_step = msg->point_step;
            merged_cloud_->row_step = 0;
        }

		// TODO: lekezelni később, hogy mit tegyen a program, ha elfogyott a memória
		if (merged_cloud_->data.capacity() < merged_cloud_->data.size() + msg->data.size())
        {
			RCLCPP_WARN(this->get_logger(), "Out of memory while merge pointclouds");
			return;
		}

		merged_cloud_->data.insert(merged_cloud_->data.end(), msg->data.begin(), msg->data.end());

		merged_cloud_->width += msg->width;
		merged_cloud_->row_step = merged_cloud_->width * merged_cloud_->point_step;

		RCLCPP_INFO(this->get_logger(), "Size of merged_cloud: %u", merged_cloud_->width);
    }

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_callback_time_).count() / 1000.0;

        if (elapsed_time < continuous_saving_rate_) {
            return;
        }

        last_callback_time_ = now;
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
        string target_frame_id = frame_id_;

        sensor_msgs::msg::PointCloud2 transformed_cloud;
        sensor_msgs::msg::PointCloud2::SharedPtr transformed_cloud_ptr;

        RCLCPP_INFO(this->get_logger(), "source (%s) target (%s)", source_frame_id.c_str(), target_frame_id.c_str());

        if (source_frame_id != "none") {
            try {
                if (tf_buffer_->canTransform(target_frame_id, source_frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0))) {
                    geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                        target_frame_id,
                        source_frame_id,
                        //msg->header.stamp
						tf2::TimePointZero
                    );

                    RCLCPP_INFO(this->get_logger(), "transformation (x=%f, y=%f, z=%f)",
                        transform_stamped.transform.translation.x,
                        transform_stamped.transform.translation.y,
                        transform_stamped.transform.translation.z
                    );

/*
					// PCL transzformációs mátrix létrehozása
					Eigen::Affine3f transform = Eigen::Affine3f::Identity();
					transform.translation() << transform_stamped.transform.translation.x,
											   transform_stamped.transform.translation.y,
											   transform_stamped.transform.translation.z;
					Eigen::Quaternionf rotation(
						transform_stamped.transform.rotation.w,
						transform_stamped.transform.rotation.x,
						transform_stamped.transform.rotation.y,
						transform_stamped.transform.rotation.z);
					transform.rotate(rotation);

					// ROS->PCL konvertálás
					pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
					pcl::fromROSMsg(*msg, pcl_cloud);

					// Pontfelhő transzformálása
					pcl::PointCloud<pcl::PointXYZ> pcl_transformed_cloud;
					pcl::transformPointCloud(pcl_cloud, pcl_transformed_cloud, transform);

					// PCL->ROS konvertálás
					pcl::toROSMsg(pcl_transformed_cloud, transformed_cloud);
					transformed_cloud.header.frame_id = target_frame_id;
*/


                    tf2::doTransform(*msg, transformed_cloud, transform_stamped);
                    transformed_cloud.header.frame_id = target_frame_id;
                    transformed_cloud_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(transformed_cloud);
                    //transformed_cloud_ptr.reset(&transformed_cloud);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Transform from %s to %s is not available yet", source_frame_id.c_str(), target_frame_id.c_str());
                    transformed_cloud_ptr = msg;
                }


            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Failed to transform PointCloud2: %s", ex.what());
            }
        } else {
            transformed_cloud_ptr = msg;
        }

        if (transformed_cloud_ptr)
            RCLCPP_INFO(this->get_logger(), "Pointer helye: %d", transformed_cloud_ptr->height);
        else
            RCLCPP_INFO(this->get_logger(), "Null a transformed_cloud_ptr pointer!");

        // Save only if the transformation exists
        if (transformed_cloud_ptr) {
            appendPointCloud(msg);
            if (!pcd_save_to_binary_file(this->get_logger(), transformed_cloud_ptr, "output.pcd")) {
                // shutdown node
                rclcpp::shutdown();
                return;
            }
        }


        if (!continuous_saving_) {
            rclcpp::shutdown();
            return;
        }
        return;

    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    string pcd_file_path_;
    string topic_name_;
    string frame_id_;
    bool dummy;
    bool continuous_saving_;
    double continuous_saving_rate_;
    int pointcloud_buffer_size_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::steady_clock::time_point last_callback_time_;

    sensor_msgs::msg::PointCloud2::SharedPtr merged_cloud_;
};
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCDSubscriber>());
    rclcpp::shutdown();
    return 0;
}
