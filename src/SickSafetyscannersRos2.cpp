
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

// Define a sensor data callback
sick::types::ScanDataCb cb = [](const sick::datastructure::Data &data) {
    
    std::cout << "Number of beams: " << data.getMeasurementDataPtr()->getNumberOfBeams() << std::endl;
};

// Create a sensor instance
//sick::AsyncSickSafetyScanner safety_scanner(sensor_ip, tcp_port, comm_settings, cb);
m_device = std::make_unique<sick::AsyncSickSafetyScanner>(sensor_ip, tcp_port, comm_settings, cb);

// Start async receiving and processing of sensor data
//safety_scanner.run();
m_device->run();
std::cout << "Running" << std::endl;

// ... Do other stuff

// Stop async processing
//safety_scanner.stop();
}
    
void SickSafetyscannersRos2::receiveUDPPaket(sick::datastructure::Data& data)
{
  std::cout << "Received UDP Packet" << std::endl;
}
