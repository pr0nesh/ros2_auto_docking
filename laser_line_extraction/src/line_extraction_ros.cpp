#include "laser_line_extraction/line_extraction_node.h"

void line_extraction::LineExtractionROS::run()
{
    // Extract the lines

    std::vector<Line> lines;

    line_extraction_.setBearingVariance(bearing_std_dev);
    line_extraction_.setRangeVariance(range_std_dev);
    line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh);
    line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh);
    line_extraction_.setMaxLineGap(max_line_gap);
    line_extraction_.setMinLineLength(min_line_length);
    line_extraction_.setMinLinePoints(min_line_points);
    line_extraction_.setMinRange(min_range);
    line_extraction_.setMinSplitDist(min_split_dist);
    line_extraction_.setOutlierDist(outlier_dist);

    line_extraction_.extractLines(lines);

    // Populate message
    laser_line_msgs::msg::LineSegmentList msg;
    populateLineSegListMsg(lines, msg);

    // publish the line
    line_publisher_->publish(msg);

    if (pub_markers_)
    {
       visualization_msgs::msg::Marker marker_msg;
       populateMarkerMsg(lines, marker_msg);
       marker_publisher_->publish(marker_msg);
    }
}


///////////////////////////////////////////////////////////////////////////////
// Cache data on first LaserScan message received
///////////////////////////////////////////////////////////////////////////////
void line_extraction::LineExtractionROS::CachedData(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    std::vector<double> bearings, cos_bearings, sin_bearings;
    std::vector<unsigned int> indices;
    
    // calculated number of laser scan of each ray
    const std::size_t num_measurements = std::ceil((scan->angle_max - scan->angle_min) / scan->angle_increment);
    
    for (std::size_t i = 0; i < num_measurements; ++i)
    {
        const double b = scan->angle_min + i * scan->angle_increment; // calculated each laser point angle
        
        bearings.push_back(b);              // store each scan angle in bearings
        cos_bearings.push_back(cos(b));     // calculated each cos(angle)
        sin_bearings.push_back(sin(b));     // calculated each sin(angle)

        indices.push_back(i);
    }

    line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
    
    std::cout << "Data has been cached." << std::endl;
}

void line_extraction::LineExtractionROS::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    // std::cout << "data_cached_\tis\t" << data_cached_ << std::endl;

    if (!data_cached_)
    {
        CachedData(scan);
        data_cached_ = true;
    }

    // std::cout << "data_cached_\tis\t" << data_cached_ << std::endl;
    std::vector<double> scan_ranges_doubles(scan->ranges.begin(), scan->ranges.end());
    line_extraction_.setRangeData(scan_ranges_doubles);
    
}

void line_extraction::LineExtractionROS::populateLineSegListMsg(const std::vector<Line>& lines, laser_line_msgs::msg::LineSegmentList& line_list_msg)
{
    // std::cout << "populateLineSegListMsg start" << std::endl;
    // std::cout << "number of line is\t" << lines.size() << std::endl;

    // start vector (x, y)
    std::vector<float> line_start(2);

    //end vector (x,y)
    std::vector<float> line_end(2);
    
    // line angel for each line
    std::vector<float> line_angle(lines.size());

    double diff_x, diff_y, angelofline;
    

    for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
    {   
        laser_line_msgs::msg::LineSegment line_msg;
        
        line_msg.angle = cit->getAngle(); 
        line_msg.radius = cit->getRadius();

        // get covariance
        line_msg.covariance[0] = cit->getCovariance()[0];
        line_msg.covariance[1] = cit->getCovariance()[1];
        line_msg.covariance[2] = cit->getCovariance()[2];
        line_msg.covariance[3] = cit->getCovariance()[3];
        
        // get line start point (x, y)
        line_msg.start[0] = cit->getStart()[0];
        line_msg.start[1] = cit->getStart()[1];
        
        // get line end point (x, y)
        line_msg.end[0] = cit->getEnd()[0];
        line_msg.end[1] = cit->getEnd()[1];

        line_list_msg.line_segments.push_back(line_msg);

    }

    line_list_msg.header.frame_id = frame_id_;
    line_list_msg.header.stamp = rclcpp::Node::now();
}

void line_extraction::LineExtractionROS::populateMarkerMsg(const std::vector<Line>& lines, visualization_msgs::msg::Marker& marker_msg)
{
    marker_msg.ns = "line_extraction";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker_msg.scale.x = 0.1;
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;
    for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
    {
        geometry_msgs::msg::Point p_start;
        p_start.x = cit->getStart()[0];
        p_start.y = cit->getStart()[1];
        p_start.z = 0;
        marker_msg.points.push_back(p_start);
        geometry_msgs::msg::Point p_end;
        p_end.x = cit->getEnd()[0];
        p_end.y = cit->getEnd()[1];
        p_end.z = 0;
        marker_msg.points.push_back(p_end);
    }

    marker_msg.header.frame_id = frame_id_;
    marker_msg.header.stamp = rclcpp::Node::now();
}
