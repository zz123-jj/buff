#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

class CameraPublisher : public rclcpp::Node
{
public:
    explicit CameraPublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("camera_publisher", options)
    {
        RCLCPP_ERROR(
            get_logger(),
            "CameraPublisher was built without libgxiapi. Install Daheng Galaxy SDK and rebuild the camera package."
        );
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(CameraPublisher)
