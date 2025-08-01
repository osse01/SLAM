#include "../include/lidar.h"

Lidar::Lidar(std::string port, int baudrate) :
    communicationChannel(sl::createSerialPortChannel(port, baudrate)),
    channel(nullptr), 
    lidar(nullptr)
{
    // Check if channel creation was successful (cast to sl_result)
    if (SL_IS_FAIL((sl_result)communicationChannel)) {
        throw std::runtime_error("Failed to create serial port channel");
    }

    channel = *communicationChannel;

    // Create a LIDAR driver instance
    auto driver_result = sl::createLidarDriver();
    if (SL_IS_FAIL((sl_result)driver_result)) {
        throw std::runtime_error("Failed to create LIDAR driver");
    }

    lidar = *driver_result;
    
    // Connect to the LIDAR
    sl_result connect_res = lidar->connect(channel);
    if (SL_IS_OK(connect_res)) {
        std::cout << "Connected to LIDAR successfully\n";

        // Check device health first
        sl_lidar_response_device_health_t healthinfo;
        connect_res = lidar->getHealth(healthinfo);
        if (SL_IS_OK(connect_res)) {
            std::cout << "Health Status: " << healthinfo.status << std::endl;
            if (healthinfo.status != SL_LIDAR_STATUS_OK) {
                throw std::runtime_error("LIDAR health check failed");
            }
        }
        
        sl_lidar_response_device_info_t deviceInfo;
        connect_res = lidar->getDeviceInfo(deviceInfo);
        if (SL_IS_OK(connect_res)) {
            std::cout << "Model: " << deviceInfo.model
                      << ", Firmware Version: " << (deviceInfo.firmware_version >> 8)
                      << "." << (deviceInfo.firmware_version & 0xffu)
                      << ", Hardware Version: " << deviceInfo.hardware_version
                      << std::endl;
        } else {
            throw std::runtime_error("Failed to get device information from LIDAR");
        }

        // Stop any previous scan
        lidar->stop();
        usleep(100000); // Wait 100ms
        
        // Start scanning with force flag
        connect_res = lidar->startScan(0, 1); // 0 = normal scan, 1 = force
        if (SL_IS_FAIL(connect_res)) {
            throw std::runtime_error("Failed to start scanning");
        }

        std::cout << "LIDAR is now scanning...\n";
        std::cout << "Press Ctrl+C to stop scanning.\n";

        // Wait for motor to spin up
        usleep(1000000); // Wait 1 second
    } else {
        throw std::runtime_error("Failed to connect to LIDAR");
    }
}

std::vector<LidarNode> Lidar::readScanData()
{

    sl_lidar_response_measurement_node_hq_t nodes[360];
    size_t count = sizeof(nodes) / sizeof(nodes[0]);
    
    // Use timeout
    sl_result grab_res = lidar->grabScanDataHq(nodes, count);
    if (grab_res == SL_RESULT_OPERATION_TIMEOUT) { // No new data available
        std::cout << "No new scan data available.\n";
        return points; // Return previous points vector
    }
    if (SL_IS_FAIL(grab_res)) {
        throw std::runtime_error("Failed to grab scan data");
    }

    

    // Sort the data
    lidar->ascendScanData(nodes, count);

    points.clear();
    points.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        if (nodes[i].dist_mm_q2 > 0) {
            LidarNode point;
            
            // Convert angle and distance
            point.angle = -(nodes[i].angle_z_q14 * 90.0f) / 16384.0f + 360.0f; // Convert to degrees
            point.distance = nodes[i].dist_mm_q2 / 4.0f;
            point.quality = nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
            
            // Filter out low quality or too distant points
            if (point.quality > 10 && point.distance < 5000) {
                points.push_back(point);
            }
        }
    }
    std::cout << "Grabbed " << points.size() << " data points\n";
    return points;
}


Lidar::~Lidar() {
    std::cout << "\nBeginning cleanup...\n";
    
    try {
        if (lidar) {
            std::cout << "Stopping LIDAR scan and motor...\n";
            lidar->stop();
            usleep(200000); // Wait 200ms for motor to stop
            
            std::cout << "Disconnecting LIDAR...\n";
            lidar->disconnect();
            
            delete lidar;
            lidar = nullptr;
        }
        
        if (channel) {
            delete channel;
            channel = nullptr;
        }
        
        std::cout << "LIDAR cleanup completed successfully.\n";
        
    } catch (...) {
        std::cerr << "Error during LIDAR cleanup, but continuing...\n";
    }
}