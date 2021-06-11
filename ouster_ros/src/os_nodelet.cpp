/**
 * @file
 * @brief Example node to publish raw sensor output on ROS topics
 *
 * ROS Parameters
 * sensor_hostname: hostname or IP in dotted decimal form of the sensor
 * udp_dest: hostname or IP where the sensor will send data packets
 * lidar_port: port to which the sensor should send lidar data
 * imu_port: port to which the sensor should send imu data
 */

#include <ros/console.h>
#include <ros/init.h>
#include <ros/ros.h>

#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#include "ouster/build.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <mrs_msgs/OusterInfo.h>

using PacketMsg   = ouster_ros::PacketMsg;
using OSConfigSrv = ouster_ros::OSConfigSrv;
namespace sensor  = ouster::sensor;

namespace ouster_nodelet
{

/* class OusterNodelet //{ */

class OusterNodelet : public nodelet::Nodelet {

public:
  virtual void onInit();
  virtual ~OusterNodelet();

private:
  void populate_metadata_defaults(sensor::sensor_info& info, sensor::lidar_mode specified_lidar_mode);
  void write_metadata(const std::string& meta_file, const std::string& metadata);
  int  run();

  ros::Publisher sensor_info_publisher_;
  std::thread    connection_loop_;

  std::string published_metadata_;
  bool is_running_ = true;
};

//}

/* populate_metadata_defaults //{ */

// fill in values that could not be parsed from metadata
void OusterNodelet::populate_metadata_defaults(sensor::sensor_info& info, sensor::lidar_mode specified_lidar_mode) {
  if (!info.name.size())
    info.name = "UNKNOWN";

  if (!info.sn.size())
    info.sn = "UNKNOWN";

  ouster::util::version v = ouster::util::version_of_string(info.fw_rev);
  if (v == ouster::util::invalid_version)
    ROS_WARN("[OusterNodelet]: Unknown sensor firmware version; output may not be reliable");
  else if (v < sensor::min_version)
    ROS_WARN("[OusterNodelet]: Firmware < %s not supported; output may not be reliable", to_string(sensor::min_version).c_str());

  if (!info.mode) {
    ROS_WARN("[OusterNodelet]: Lidar mode not found in metadata; output may not be reliable");
    info.mode = specified_lidar_mode;
  }

  if (!info.prod_line.size())
    info.prod_line = "UNKNOWN";

  if (info.beam_azimuth_angles.empty() || info.beam_altitude_angles.empty()) {
    ROS_WARN("[OusterNodelet]: Beam angles not found in metadata; using design values");
    info.beam_azimuth_angles  = sensor::gen1_azimuth_angles;
    info.beam_altitude_angles = sensor::gen1_altitude_angles;
  }
}

//}

/*  write_metadata() //{ */

// try to write metadata file
void OusterNodelet::write_metadata(const std::string& meta_file, const std::string& metadata) {
  std::ofstream ofs;
  ofs.open(meta_file);
  ofs << metadata << std::endl;
  ofs.close();
  if (ofs) {
    ROS_INFO("[OusterNodelet]: Wrote metadata to $ROS_HOME/%s", meta_file.c_str());
  } else {
    ROS_WARN("[OusterNodelet]: Failed to write metadata to %s; check that the path is valid", meta_file.c_str());
  }
}

// try to write metadata file
void write_metadata(const std::string& meta_file, const std::string& metadata) {
  std::ofstream ofs;
  ofs.open(meta_file);
  ofs << metadata << std::endl;
  ofs.close();
  if (ofs) {
    ROS_INFO("[OusterNodelet]: Wrote metadata to $ROS_HOME/%s", meta_file.c_str());
  } else {
    ROS_WARN("[OusterNodelet]: Failed to write metadata to %s; check that the path is valid", meta_file.c_str());
  }
}

//}

/* run() //{ */

int OusterNodelet::run() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  ROS_INFO("[OusterNodelet] Initialization started.");

  sensor_info_publisher_ = nh.advertise<mrs_msgs::OusterInfo>("sensor_info", 1, true);

  // empty indicates "not set" since roslaunch xml can't optionally set params
  auto hostname           = nh.param("sensor_hostname", std::string{});
  auto udp_dest           = nh.param("udp_dest", std::string{});
  auto lidar_port         = nh.param("lidar_port", 0);
  auto imu_port           = nh.param("imu_port", 0);
  auto replay             = nh.param("replay", false);
  auto lidar_mode_arg     = nh.param("lidar_mode", std::string{});
  auto timestamp_mode_arg = nh.param("timestamp_mode", std::string{});

  // fall back to metadata file name based on hostname, if available
  auto meta_file = nh.param("metadata", std::string{});
  if (!meta_file.size() && hostname.size())
    meta_file = hostname + ".json";

  // set lidar mode from param
  sensor::lidar_mode lidar_mode = sensor::MODE_UNSPEC;
  if (lidar_mode_arg.size()) {
    if (replay)
      ROS_WARN("[OusterNodelet]: Lidar mode set in replay mode. May be ignored");

    lidar_mode = sensor::lidar_mode_of_string(lidar_mode_arg);
    if (!lidar_mode) {
      ROS_ERROR("[OusterNodelet]: Invalid lidar mode %s", lidar_mode_arg.c_str());
      return 0;
    }
  }

  // set timestamp mode from param
  sensor::timestamp_mode timestamp_mode = sensor::TIME_FROM_UNSPEC;
  if (timestamp_mode_arg.size()) {
    if (replay)
      ROS_WARN("[OusterNodelet]: Timestamp mode set in replay mode. Will be ignored");

    timestamp_mode = sensor::timestamp_mode_of_string(timestamp_mode_arg);
    if (!timestamp_mode) {
      ROS_ERROR("[OusterNodelet]: Invalid timestamp mode %s", timestamp_mode_arg.c_str());
      return 0;
    }
  }

  if (!replay && (!hostname.size() || !udp_dest.size())) {
    ROS_ERROR("[OusterNodelet]: Must specify both hostname and udp destination");
    return 0;
  }

  ROS_INFO("[OusterNodelet]: Client version: %s", ouster::CLIENT_VERSION_FULL);

  if (replay) {
    ROS_INFO("[OusterNodelet]: Running in replay mode");

    // populate info for config service
    try {
      auto info          = sensor::metadata_from_json(meta_file);
      published_metadata_ = to_string(info);

      ROS_INFO("[OusterNodelet]: Using lidar_mode: %s", sensor::to_string(info.mode).c_str());
      ROS_INFO("[OusterNodelet]: %s sn: %s firmware rev: %s", info.prod_line.c_str(), info.sn.c_str(), info.fw_rev.c_str());

      // just serve config service
      /* ros::spin(); */
      /* return 0; */

      ROS_INFO("[OsNodelet] Advertising service os_config.");
      auto srv = nh.advertiseService<OSConfigSrv::Request, OSConfigSrv::Response>("os_config", [&](OSConfigSrv::Request&, OSConfigSrv::Response& res) {
        if (published_metadata_.size()) {
          res.metadata = published_metadata_;
          return true;
        } else {
          ROS_ERROR("returning false");
          return false;
        }
      });
      ROS_INFO("[OsNodelet] Service os_config advertised.");

      while (ros::ok())
        ros::Duration(0.1).sleep();
    }
    catch (const std::runtime_error& e) {
      ROS_ERROR("[OusterNodelet]: Error when running in replay mode: %s", e.what());
    }
  } else {
    ROS_INFO("[OusterNodelet]: Connecting to %s; sending data to %s", hostname.c_str(), udp_dest.c_str());
    ROS_INFO("[OusterNodelet]: Waiting for sensor to initialize ...");

    auto cli = sensor::init_client(hostname, udp_dest, lidar_mode, timestamp_mode, lidar_port, imu_port);

    if (!cli) {
      ROS_ERROR("[OusterNodelet]: Failed to initialize sensor at: %s", hostname.c_str());
      ros::requestShutdown();
    }

    ROS_INFO("[OusterNodelet]: Sensor initialized successfully");

    // write metadata file to cwd (usually ~/.ros)
    auto metadata = sensor::get_metadata(*cli);
    write_metadata(meta_file, metadata);

    // populate sensor info
    auto info = sensor::parse_metadata(metadata);
    populate_metadata_defaults(info, sensor::MODE_UNSPEC);
    published_metadata_ = to_string(info);

    mrs_msgs::OusterInfo ouster_info;
    ouster_info.name                           = info.name;
    ouster_info.sn                             = info.sn;
    ouster_info.fw_rev                         = info.fw_rev;
    ouster_info.mode                           = info.mode;
    ouster_info.prod_line                      = info.prod_line;
    ouster_info.beam_azimuth_angles            = info.beam_azimuth_angles;
    ouster_info.beam_altitude_angles           = info.beam_altitude_angles;
    ouster_info.lidar_origin_to_beam_origin_mm = info.lidar_origin_to_beam_origin_mm;

    info.imu_to_sensor_transform.transposeInPlace();
    info.lidar_to_sensor_transform.transposeInPlace();
    info.extrinsic.transposeInPlace();

    for (int i = 0; i < info.imu_to_sensor_transform.size(); i++) {
      ouster_info.imu_to_sensor_transform.push_back(info.imu_to_sensor_transform(i));
    }

    for (int i = 0; i < info.lidar_to_sensor_transform.size(); i++) {
      ouster_info.lidar_to_sensor_transform.push_back(info.lidar_to_sensor_transform(i));
    }

    for (int i = 0; i < info.extrinsic.size(); i++) {
      ouster_info.extrinsic.push_back(info.extrinsic(i));
    }

    try {
      sensor_info_publisher_.publish(ouster_info);
      ROS_INFO("[OusterNodelet]: Sensor info published!");
    }
    catch (...) {
      ROS_ERROR_THROTTLE(1.0, "[OusterNodelet]: could not publish topic %s", sensor_info_publisher_.getTopic().c_str());
    }

    ROS_INFO("[OusterNodelet]: Hostname: %s", info.name.c_str());
    ROS_INFO("[OusterNodelet]: Using lidar_mode: %s", sensor::to_string(info.mode).c_str());
    ROS_INFO("[OusterNodelet]: %s sn: %s firmware rev: %s", info.prod_line.c_str(), info.sn.c_str(), info.fw_rev.c_str());

    auto lidar_packet_pub = nh.advertise<PacketMsg>("lidar_packets", 1280);
    auto imu_packet_pub   = nh.advertise<PacketMsg>("imu_packets", 100);

    auto pf = sensor::get_format(info);

    PacketMsg lidar_packet, imu_packet;
    lidar_packet.buf.resize(pf.lidar_packet_size + 1);
    imu_packet.buf.resize(pf.imu_packet_size + 1);

    ROS_INFO("[OsNodelet] Advertising service os_config.");
    auto srv = nh.advertiseService<OSConfigSrv::Request, OSConfigSrv::Response>("os_config", [&](OSConfigSrv::Request&, OSConfigSrv::Response& res) {
      if (published_metadata_.size()) {
        res.metadata = published_metadata_;
        return true;
      } else {
        ROS_ERROR("returning false");
        return false;
      }
    });
    ROS_INFO("[OsNodelet] Service os_config advertised.");

    while (ros::ok() && is_running_) {
      auto state = sensor::poll_client(*cli);
      if (state == sensor::EXIT) {
        ROS_INFO("[OusterNodelet]: poll_client: caught signal, exiting");
        return EXIT_SUCCESS;
      }
      if (state & sensor::CLIENT_ERROR) {
        ROS_ERROR("[OusterNodelet]: poll_client: returned error");
        return EXIT_FAILURE;
      }
      if (state & sensor::LIDAR_DATA) {
        if (sensor::read_lidar_packet(*cli, lidar_packet.buf.data(), pf))
          lidar_packet_pub.publish(lidar_packet);
      }
      if (state & sensor::IMU_DATA) {
        if (sensor::read_imu_packet(*cli, imu_packet.buf.data(), pf))
          imu_packet_pub.publish(imu_packet);
      }

    }
  }
  return EXIT_SUCCESS;
}

//}

/* onInit() //{ */

void OusterNodelet::onInit() {

  connection_loop_ = std::thread(&OusterNodelet::run, this);

}

//}

/* ~OusterNodelet */ /*//{*/

OusterNodelet::~OusterNodelet() {
  is_running_ = false;
  connection_loop_.join();
}

/*//}*/

}  // namespace ouster_nodelet

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(ouster_nodelet::OusterNodelet, nodelet::Nodelet);
