#include "laser_line_extraction/line_extraction_node.h"

int main(int argc, char* argv[])
{
    std::shared_ptr<line_extraction::LineExtractionROS> line_extraction_node;

    rclcpp::init(argc, argv);

    line_extraction_node = std::make_shared<line_extraction::LineExtractionROS>();

    RCLCPP_INFO(line_extraction_node->get_logger(), "[%s] line_extraction_node has been started", __func__);

    rclcpp::WallRate rate(50.0);

    while (rclcpp::ok()) 
    {
        line_extraction_node->run();
        rclcpp::spin_some(line_extraction_node);
        rate.sleep();
    }
    
    return 0;
}
