#include<sick_safetyscanners_base/SickSafetyscanners.h>
#include<sick_safetyscanners_base/datastructure/Data.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SickSafetyscannersRos2 : public rclcpp::Node
{
  public:
    SickSafetyscannersRos2();
    void receiveUDPPaket(sick::datastructure::Data& data);
    
  private:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    std::unique_ptr<sick::AsyncSickSafetyScanner> m_device;
    sick::datastructure::CommSettings m_communications_settings;
};
