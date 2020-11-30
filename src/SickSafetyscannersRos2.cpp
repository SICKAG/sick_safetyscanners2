
#include <sick_safetyscanners2/SickSafetyscannersRos2.h>

SickSafetyscannersRos2::SickSafetyscannersRos2()
  : Node("SickSafetyscannersRos2")
{
  std::cout << "Init ROS2 Node" << std::endl;



  // Sensor IP and Port
sick::types::ip_address_t sensor_ip = boost::asio::ip::address_v4::from_string("192.168.1.11");
sick::types::port_t tcp_port {2122};

// Prepare the CommSettings for Sensor streaming data
sick::datastructure::CommSettings comm_settings;
comm_settings.host_ip = boost::asio::ip::address_v4::from_string("192.168.1.9");
comm_settings.host_udp_port = 0;


// Bind callback
std::function< void(const sick::datastructure::Data&) > callback = std::bind( &SickSafetyscannersRos2::receiveUDPPaket, this, std::placeholders::_1 );



// Create a sensor instance
m_device = std::make_unique<sick::AsyncSickSafetyScanner>(sensor_ip, tcp_port, comm_settings, callback);

// Start async receiving and processing of sensor data
m_device->run();
std::cout << "Running" << std::endl;

}
    
void SickSafetyscannersRos2::receiveUDPPaket(const sick::datastructure::Data& data)
{
  std::cout << "Received UDP Packet" << std::endl;
  std::cout << "Number of beams: " << data.getMeasurementDataPtr()->getNumberOfBeams() << std::endl;
}
