#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <string.h>
#include <stdio.h>
#include <thread>
#include <memory>
#include <regex>
#include <array>
#include "manager_pepper/Ping.h"
#include "manager_pepper/PingStatus.h"
#include <numeric>
void ping(std::shared_ptr<std::array<uint32_t, 20>> packet_loss, std::shared_ptr<std::float_t> rtt_min,
          std::shared_ptr<std::float_t> rtt_avg, std::shared_ptr<std::float_t> rtt_max,
          std::shared_ptr<std::float_t> rtt_mdev, std::shared_ptr<std::uint8_t> connection_status) {
  // start plan listener
  char buf[32];
  // FILE* p = popen("netstat --tcp | awk \'$5 !~ \"(Pepper2|localhost|0\\.0\\.0\\.0):[0-9*]+\"\'", "r");
  std::uint8_t packet_loss_index = 0;
  while (ros::ok()) {
    FILE* p = popen("ping -c 10 -i 0.2 -W 4 192.168.50.44", "r");
    std::string s;
    for (size_t count; (count = fread(buf, 1, sizeof(buf), p));) {
      s += std::string(buf, buf + count);
    }
    pclose(p);
    std::regex loss_regex("\\d+(?=\\% packet loss)");
    std::regex rtt_regex("([\\d\\.]+)/(\\d+\\.\\d+)/(\\d+\\.\\d+)/(\\d+\\.\\d+)");
    std::smatch match;

    if (std::regex_search(s, match, loss_regex)) {
      // the match is a pourcentage so we want to convert it to the amount of package we lost
      int packet_loss_pourcent = std::stoi(match.str());
      if (packet_loss_pourcent == 100) {
        *connection_status = 0;
      } else {
        *connection_status = 1;
      }
      int packet_loss_nb = (packet_loss_pourcent * 10) / 100;
      std::cout << "Number of packet lost : " << packet_loss_nb << '\n';
      (*packet_loss)[packet_loss_index % (*packet_loss).max_size()] = packet_loss_nb;
    }
    if (std::regex_search(s, match, rtt_regex)) {
      *rtt_min = std::stod(match[1].str());
      *rtt_avg = std::stod(match[2].str());
      *rtt_max = std::stod(match[3].str());
      *rtt_mdev = std::stod(match[4].str());
    }
    packet_loss_index++;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "robobreizh_manager");

  // Connect SQLite database
  std::string db_file_path = "/home/nao/robobreizh_pepper_ws/src/manager_pepper/manager_db/roboBreizhDb.db";

  // Create ros node
  ros::NodeHandle nh;
  ros::Publisher ping_publisher = nh.advertise<manager_pepper::Ping>("/robobreizh/packet_loss", 1000);
  ros::Rate rate(1);

  auto packet_loss = std::make_shared<std::array<uint32_t, 20>>();
  auto rtt_min = std::make_shared<std::float_t>();
  auto rtt_avg = std::make_shared<std::float_t>();
  auto rtt_max = std::make_shared<std::float_t>();
  auto rtt_mdev = std::make_shared<std::float_t>();
  auto connection_status = std::make_shared<std::uint8_t>();

  std::thread ping_thread(ping, packet_loss, rtt_min, rtt_avg, rtt_max, rtt_mdev, connection_status);
  while (ros::ok()) {
    manager_pepper::Ping ping_msg;
    // packet_loss is the pourcentage of packet loss over 20 iterations
    ping_msg.packet_loss_pourcent =
        std::accumulate((*packet_loss).begin(), (*packet_loss).end(), 0) * 100 / ((*packet_loss).size() * 10);
    ping_msg.rtt_min = *rtt_min.get();
    ping_msg.rtt_avg = *rtt_avg.get();
    ping_msg.rtt_max = *rtt_max.get();
    ping_msg.rtt_mdev = *rtt_mdev.get();
    ping_msg.ping_status.status = *connection_status.get();
    ping_msg.header.stamp = ros::Time::now();
    ping_publisher.publish(ping_msg);
    rate.sleep();
  }
  ping_thread.join();

  return 0;
}