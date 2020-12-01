#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include<sick_safetyscanners_base/SickSafetyscanners.h>

#include <sick_safetyscanners2/SickSafetyscannersRos2.h>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world sick_safetyscanners2 package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sick::SickSafetyscannersRos2>());
  rclcpp::shutdown();
  return 0;
}
