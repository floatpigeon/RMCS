#include <memory>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <string>

class PointCloudSave : public rclcpp::Node {
    using PointT      = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;

public:
    PointCloudSave()
        : Node("pointcloud_save")
        , logger_(get_logger()) {
        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", rclcpp::SystemDefaultsQoS(),
            std::bind(&PointCloudSave::subscriber_callback, this, std::placeholders::_1));

        integral_pointcloud_ = std::make_shared<PointCloudT>();

        output_filename_ =
            "/workspaces/RMCS/rmcs_ws/src/rmcs_dart_guide/src/point_cloud_files/milkdragon_test_pointcloud.pcd";
        // output_filename_ =
        //     "/workspaces/RMCS/rmcs_ws/src/rmcs_dart_guide/src/point_cloud_files/record_original_pointcloud.pcd";
        RCLCPP_INFO(logger_, "Node launched,waiting for data");
    }

private:
    void subscriber_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        PointCloudT::Ptr received_cloud(new PointCloudT());
        pcl::fromROSMsg(*msg, *received_cloud);

        if (pointcloud_frame_count_ == 0) {
            *integral_pointcloud_ = *received_cloud;
        } else {
            *integral_pointcloud_ += *received_cloud;
        }
        pointcloud_frame_count_++;

        if (pointcloud_frame_count_ == 20) {

            // pcl::PointCloud<pcl::PointXYZ>::Ptr box_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            // box_filter(integral_pointcloud_, box_filtered_cloud);
            // int return_status = pcl::io::savePCDFileBinary(output_filename_, *box_filtered_cloud);

            int return_status = pcl::io::savePCDFileBinary(output_filename_, *integral_pointcloud_);

            if (return_status == 0) {
                RCLCPP_INFO(this->get_logger(), "Successfully saved %s", output_filename_.c_str());
            } else {
                RCLCPP_ERROR(
                    this->get_logger(), "Failed to save %s (Error code: %d)", output_filename_.c_str(),
                    return_status);
            }
        } else if (pointcloud_frame_count_ >= 20) {
            RCLCPP_INFO(logger_, "pcd file save compelete, now can exit");
            pointcloud_subscriber_.reset();
        }
    }

    static void
        box_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& output) {

        // pcl::PointCloud<pcl::PointXYZ>::Ptr box_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setInputCloud(input);

        // 基地大致距离
        // Eigen::Vector4f min_point(25.0, -2.5, -0.5, 1.0);
        // Eigen::Vector4f max_point(30.0, 2.5, 0.5, 1.0);

        // TEST
        Eigen::Vector4f min_point(0.0, -2.5, -0.75, 1.0);
        Eigen::Vector4f max_point(5.0, 2.5, 0.75, 1.0);

        box_filter.setMin(min_point);
        box_filter.setMax(max_point);
        box_filter.setNegative(false);
        box_filter.filter(*output);
    }
    rclcpp::Logger logger_;

    std::string output_filename_;
    int pointcloud_frame_count_ = 0;
    PointCloudT::Ptr integral_pointcloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudSave>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}