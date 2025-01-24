// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: LD Robot, Will Son

#include <stdio.h>
#include <iostream>
#include "../include/cmd_interface_linux.h"
#include "../include/lipkg.h"
#include "../include/transform.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("laser_scan_publisher");
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub;

  LiPkg * pkg;
  std::string product;
  int32_t ver = 8;
  pkg = new LD08_LiPkg;

  CmdInterfaceLinux cmd_port(ver);
  std::vector<std::pair<std::string, std::string>> device_list;
  std::string port_name;
  cmd_port.GetCmdDevices(device_list);
  for (auto n : device_list) {
    std::cout << n.first << "    " << n.second << std::endl;
    if (strstr(n.second.c_str(), "CP2102")) {
      port_name = n.first;
    }
  }

  if (port_name.empty() == false) {
    std::cout << "FOUND LDS-02" << product << std::endl;
    cmd_port.SetReadCallback(
      [&pkg](const char * byte, size_t len) {
        if (pkg->Parse((const uint8_t *)(byte), len)) {
          pkg->AssemblePacket();
        }
      });

    if (cmd_port.Open(port_name)) {
      std::cout << "LDS-02" << product << " started successfully " << std::endl;
    }
    
    // char topic_name[20]={0};
    // strcat(topic_name,product.c_str());
    // strcat(topic_name,"/LDLiDAR");
    lidar_pub = node->create_publisher<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::QoS(rclcpp::SensorDataQoS())
    );

    // This loop continuously runs as long as the rclcpp::ok() is true
    while (rclcpp::ok()) {
      if (pkg->IsFrameReady()) { // If a frame is ready by calling the method on the pkg object
        pkg->setStamp(node->now()); // Set the timestamp of the package

        auto scan_msg = pkg->GetLaserScan(); //scan_msg is a sensor_msgs::msg::LaserScan object, which should have the lidar scan readings
        
        // Limit to 40 lidar readings

        int step = scan_msg.ranges.size() / 40; // depending on the size of the scan_msg array, the step is calculated to get 40 readings

        std::vector<float> sparsed_ranges;
        std::vector<float> sparsed_intensities;

        for (int i = 0; i < 40; ++i) {
          sparsed_ranges.push_back(scan_msg.ranges[i * step]);
          sparsed_intensities.push_back(scan_msg.intensities[i * step]);
        }

        scan_msg.ranges = sparsed_ranges;
        scan_msg.intensities = sparsed_intensities;

        // Adjust the angle_increment parameter
        // scan_msg.angle_increment = scan_msg.angle_increment * step; // Attempt #1
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / 40.0; // Attempt #2

        // Check if any of the values are Nan, if so, replace with the average of lidar readings
        for (int i = 0; i < scan_msg.ranges.size(); ++i) {
          if (std::isnan(scan_msg.ranges[i])) {
            scan_msg.ranges[i] = (scan_msg.range_min + scan_msg.range_max) / 2.0;
          }
        }
        
        lidar_pub->publish(scan_msg); // Publish the lidar scan readings limited to 40 readings
        pkg->ResetFrameReady();
      }
    }

  // ORIGINAL CODE
  // while (rclcpp::ok()) {
  //     if (pkg->IsFrameReady()) { // If a frame is ready by calling the method on the pkg object
  //       pkg->setStamp(node->now()); // Set the timestamp of the package
  //       lidar_pub->publish(pkg->GetLaserScan()); // GetLaserScan retrieves the lidar scan readings
  //       pkg->ResetFrameReady();
  //     }
  //   }
  } else {
    std::cout << "Can't find LDS-02" << product << std::endl;
  }

  return 0;
}
