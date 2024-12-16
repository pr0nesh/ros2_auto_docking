#ifndef LINE_EXTRACTION_ROS_H
#define LINE_EXTRACTION_ROS_H

#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "laser_line_msgs/msg/line_segment_list.hpp"
#include "laser_line_msgs/msg/line_segment.hpp"
#include "line.h"
#include "line_extraction.h"

namespace line_extraction 

{
    class LineExtractionROS : public rclcpp::Node
    {
        public:

            LineExtractionROS(): Node("line_extraction")
            {

                // Parameters used by this node

                this->declare_parameter<bool>("data_cached", false);
                this->get_parameter("data_cached", data_cached_);  

                this->declare_parameter<std::string>("frame_id", "laser_frame");
                this->get_parameter("frame_id", frame_id_);
                RCLCPP_INFO(this->get_logger(), "[%s] frame_id = %s", __func__, frame_id_.c_str());

                this->declare_parameter<std::string>("scan_topic", "scan");
                this->get_parameter("scan_topic", scan_topic_);
                RCLCPP_INFO(this->get_logger(), "[%s] scan_topic = %s", __func__, scan_topic_.c_str());

                this->declare_parameter<bool>("publish_markers", true);
                this->get_parameter("publish_markers", pub_markers_);
                RCLCPP_INFO(this->get_logger(), "[%s] publish_markers = %d", __func__,  pub_markers_);

                 this->declare_parameter<float>("frequency", 50);
                 this->get_parameter("frequency", frequency_);
                 RCLCPP_INFO(this->get_logger(), "[%s] frequency = %f", __func__,  frequency_);

                // Parameters used by the line extraction algorithm

                this->declare_parameter<double>("bearing_std_dev", 1e-3);
                this->get_parameter("bearing_std_dev", bearing_std_dev);
                RCLCPP_INFO(this->get_logger(), "[%s] bearing_std_dev = %f", __func__, bearing_std_dev);

                this->declare_parameter<double>("range_std_dev", 0.01);
                this->get_parameter("range_std_dev", range_std_dev);
                RCLCPP_INFO(this->get_logger(), "[%s] range_std_dev = %f", __func__, range_std_dev);

                this->declare_parameter<double>("least_sq_angle_thresh", 1e-4);
                this->get_parameter("least_sq_angle_thresh", least_sq_angle_thresh);
                RCLCPP_INFO(this->get_logger(), "[%s] least_sq_angle_thresh = %f", __func__, least_sq_angle_thresh);

                this->declare_parameter<double>("least_sq_radius_thresh", 1e-4);
                this->get_parameter("least_sq_radius_thresh", least_sq_radius_thresh);
                RCLCPP_INFO(this->get_logger(), "[%s] least_sq_radius_thresh = %f", __func__, least_sq_radius_thresh);

                this->declare_parameter<double>("max_line_gap", 5.0);
                this->get_parameter("max_line_gap", max_line_gap);
                RCLCPP_INFO(this->get_logger(), "[%s] max_line_gap = %f", __func__, max_line_gap);

                this->declare_parameter<double>("min_line_length", 0.03);
                this->get_parameter("min_line_length", min_line_length);
                RCLCPP_INFO(this->get_logger(), "[%s] min_line_length = %f", __func__, min_line_length);

                this->declare_parameter<double>("min_range", 0.2);
                this->get_parameter("min_range", min_range);
                RCLCPP_INFO(this->get_logger(), "[%s] min_range = %f", __func__, min_range);

                

                this->declare_parameter<double>("min_split_dist", 0.05);
                this->get_parameter("min_split_dist", min_split_dist);
                RCLCPP_INFO(this->get_logger(), "[%s] min_split_dist = %f", __func__, min_split_dist);

                this->declare_parameter<double>("outlier_dist", 0.05);
                this->get_parameter("outlier_dist", outlier_dist);
                RCLCPP_INFO(this->get_logger(), "[%s] outlier_dist = %f", __func__, outlier_dist);

                this->declare_parameter<int>("min_line_points", 5);
                this->get_parameter("min_line_points", min_line_points);
                RCLCPP_INFO(this->get_logger(), "[%s] min_line_points = %d", __func__, min_line_points);

                scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_,  
                                                                                      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(), 
                                                                                      std::bind(&LineExtractionROS::scan_cb, this, std::placeholders::_1));
                line_publisher_ = this->create_publisher<laser_line_msgs::msg::LineSegmentList>("line_segments", 1);
                marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("line_markers", 1);


            }

            void run();


        private:

            LineExtraction line_extraction_;

            bool data_cached_, pub_markers_;
            std::string frame_id_;
            std::string scan_topic_;
            float frequency_;
            double bearing_std_dev, range_std_dev, least_sq_angle_thresh, least_sq_radius_thresh, max_line_gap, min_line_length, min_range, max_range, min_split_dist, outlier_dist;
            int min_line_points;

            void populateLineSegListMsg(const std::vector<Line>&, laser_line_msgs::msg::LineSegmentList&);
            void populateMarkerMsg(const std::vector<Line>&, visualization_msgs::msg::Marker&);
            void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr scan);
            void CachedData(const sensor_msgs::msg::LaserScan::SharedPtr scan);
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
            rclcpp::Publisher<laser_line_msgs::msg::LineSegmentList>::SharedPtr line_publisher_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    };
}

#endif
