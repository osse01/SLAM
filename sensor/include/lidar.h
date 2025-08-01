#ifndef LIDAR_H
#define LIDAR_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <unistd.h>


#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

struct LidarNode {
    float angle, distance;
    int quality;
};

class Lidar {
   
    public:
        Lidar(std::string port = "/dev/ttyUSB0", int baudrate = 460800);
        ~Lidar();

        /*
        Scan one revolution of data and return the points.
        */
        std::vector<LidarNode> readScanData();
    private:
        sl::Result<sl::IChannel*> communicationChannel;
        sl::IChannel* channel;
        sl::ILidarDriver* lidar;
        std::vector<LidarNode> points;
};

#endif // LIDAR_H