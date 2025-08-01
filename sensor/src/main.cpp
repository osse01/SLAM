#include "lidar.h"
#include <csignal>
#include <atomic>
#include <iomanip>
#include <fstream>

std::atomic<bool> running(true);

void signalHandler(int signum) {
    running = false;
}


int main() {
    // Register signal handler for Ctrl+C (SIGINT)
    signal(SIGINT, signalHandler);
    try {
        Lidar lidar;
        while (running) {
            auto points = lidar.readScanData();
            static bool use_file_a = true;
            if (!points.empty()) {
                // for (const auto& point : points) {
                //     std::cout << std::fixed     << std::setprecision(2)
                //               <<   "Angle: "    << std::setw(8) << point.angle 
                //               << ", Distance: " << std::setw(8) << point.distance 
                //               << ", Quality: "  << std::setw(3) << point.quality << "\n";
                // }

                // Save to file for gnuplot
                std::string filename = use_file_a ? "lidar_data_a.txt" : "lidar_data_b.txt";
                std::ofstream datafile(filename);
               
                for (const auto& point : points) {
                    datafile << std::fixed << std::setprecision(2)
                             << point.angle << " " 
                             << point.distance << " " 
                             << point.quality << "\n";
                }
                datafile.close();

                // Update symlink to current file
                unlink("lidar_data.txt");
                symlink(filename.c_str(), "lidar_data.txt");
                use_file_a = !use_file_a;
            } else {
                std::cout << "No points read.\n";
            }
            usleep(700000); // Sleep for 500ms between scans
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
        }
    std::cout << "Program terminated successfully.\n";
    return 0;
}