/**
 * @file
 * @brief Example node to publish point clouds and imu topics
 */

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>

#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <mutex>

using PacketMsg         = ouster_ros::PacketMsg;
using PacketMsgConstPtr = ouster_ros::PacketMsgConstPtr;
using Cloud             = ouster_ros::Cloud;
using Point             = ouster_ros::Point;
namespace sensor        = ouster::sensor;


namespace ouster_nodelet
{

/* class OusterCloudNodelet //{ */

class OusterCloudNodelet : public nodelet::Nodelet {

public:
  virtual void onInit();
  virtual ~OusterCloudNodelet();

private:
  bool                                is_initialized_ = false;
  ros::Subscriber                     lidar_packet_sub_, imu_packet_sub_;
  ros::Publisher                      lidar_pub_, imu_pub_, is_alive_pub_;
  tf2_ros::StaticTransformBroadcaster tf_bcast_;

  sensor::sensor_info                    info_;
  uint32_t                               H_;
  uint32_t                               W_;
  std::shared_ptr<sensor::packet_format> pf_ptr_;
  std::mutex                             mutex_pf_;

  ouster::XYZLut    xyz_lut_;
  ouster::LidarScan ls_;

  std::unique_ptr<ouster::ScanBatcher> batch_ptr_;

  std::string sensor_frame_;
  std::string imu_frame_;
  std::string lidar_frame_;

  std::thread init_thread_;
  int         run();

  bool use_system_timestamp_;
};

//}

/* run() */ /*//{*/

int OusterCloudNodelet::run() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  ROS_INFO("[OsCloudNodelet] Initializing.");

  auto tf_prefix        = nh.param("tf_prefix", std::string{});
  use_system_timestamp_ = nh.param("use_system_timestamp", false);
  ROS_INFO("[OusterCloudNodelet] tf_prefix: %s", tf_prefix.c_str());
  if (!tf_prefix.empty() && tf_prefix.back() != '/')
    tf_prefix.append("/");
  sensor_frame_ = tf_prefix + "os_sensor";
  imu_frame_    = tf_prefix + "os_imu";
  lidar_frame_  = tf_prefix + "os_lidar";

  auto fixed_frame = nh.param("fixed_frame_id", std::string{});
  if (!fixed_frame.empty())
    fixed_frame = tf_prefix + fixed_frame;
  tf::TransformListener listener;
  auto                  waitForTransform = nh.param("wait_for_transform", 0.01);

  ouster_ros::OSConfigSrv cfg{};
  auto                    client = nh.serviceClient<ouster_ros::OSConfigSrv>("os_config");
  client.waitForExistence();
  if (!client.call(cfg)) {
    ROS_ERROR("[OusterCloudNodelet]: Calling config service failed");
    return false;
  }

  info_ = sensor::parse_metadata(cfg.response.metadata);
  H_    = info_.format.pixels_per_column;
  W_    = info_.format.columns_per_frame;

  pf_ptr_ = std::make_shared<sensor::packet_format>(sensor::get_format(info_));

  lidar_pub_    = nh.advertise<sensor_msgs::PointCloud2>("points", 10);
  is_alive_pub_ = nh.advertise<std_msgs::Bool>("is_alive", 10);
  imu_pub_      = nh.advertise<sensor_msgs::Imu>("imu", 100);

  xyz_lut_ = ouster::make_xyz_lut(info_);

  ls_ = ouster::LidarScan{W_, H_};

  batch_ptr_ = std::make_unique<ouster::ScanBatcher>(W_, *pf_ptr_);

  // callback for lidar packages
  auto lidar_handler = [&](const PacketMsgConstPtr& pm) mutable {
    if (!is_initialized_) {
      return;
    }
    std::scoped_lock lock(mutex_pf_);
    if (batch_ptr_->operator()(pm->buf.data(), ls_)) {
      auto h = std::find_if(ls_.headers.begin(), ls_.headers.end(), [](const auto& h) { return h.timestamp != std::chrono::nanoseconds{0}; });
      if (h != ls_.headers.end()) {
        Cloud cloud{W_, H_};

        if (fixed_frame.empty()) {
          scan_to_cloud(xyz_lut_, h->timestamp, ls_, cloud);
        } else {
          scan_to_cloud(xyz_lut_, h->timestamp, ls_, cloud, listener, fixed_frame, sensor_frame_, waitForTransform);
        }

        sensor_msgs::PointCloud2 msg = ouster_ros::cloud_to_cloud_msg(cloud, h->timestamp, sensor_frame_);
        /* sensor_msgs::PointCloud2 msg = ouster_ros::cloud_to_cloud_msg(cloud, h->timestamp, lidar_frame_); */
        if (use_system_timestamp_) {
          // if packets are not PTP-timestamped, then the header is the time since the sensor was initialized, rather than the time since the epoch
          msg.header.stamp = ros::Time::now();
        }
        lidar_pub_.publish(msg);

        std_msgs::Bool alive_msg;
        alive_msg.data = true;
        is_alive_pub_.publish(alive_msg);

        /* lidar_pub_.publish(ouster_ros::cloud_to_cloud_msg(cloud, h->timestamp, sensor_frame_)); */
        ROS_INFO_THROTTLE(10.0, "[OusterCloudNodelet]: publishing point cloud");
      }
    }
  };

  // callback for imu packages
  auto imu_handler = [&](const PacketMsgConstPtr& p) {
    if (!is_initialized_) {
      return;
    }
    imu_pub_.publish(ouster_ros::packet_to_imu_msg(*p, imu_frame_, *pf_ptr_));
    ROS_INFO_THROTTLE(10.0, "[OusterCloudNodelet]: publishing imu data");
  };


  lidar_packet_sub_ = nh.subscribe<PacketMsg, const PacketMsgConstPtr&>("lidar_packets", 2048, lidar_handler);
  imu_packet_sub_   = nh.subscribe<PacketMsg, const PacketMsgConstPtr&>("imu_packets", 100, imu_handler);

  // publish transforms
  tf_bcast_ = tf2_ros::StaticTransformBroadcaster();

  tf_bcast_.sendTransform(ouster_ros::transform_to_tf_msg(info_.imu_to_sensor_transform, sensor_frame_, imu_frame_));

  tf_bcast_.sendTransform(ouster_ros::transform_to_tf_msg(info_.lidar_to_sensor_transform, sensor_frame_, lidar_frame_));

  is_initialized_ = true;

  ROS_INFO("[OsCloudNodelet] Finished initialization.");

  return true;
}

/*//}*/

/* onInit() //{ */

void OusterCloudNodelet::onInit() {

  // initialize everything on separate thread so that the wait for os_config service does not block other nodelets
  init_thread_ = std::thread(&OusterCloudNodelet::run, this);
}

//}

/* ~OusterCloudNodelet */ /*//{*/

OusterCloudNodelet::~OusterCloudNodelet() {
  ROS_INFO("OusterCloudNodelet destructor called");
  init_thread_.join();
  ROS_INFO("OusterCloudNodelet destructor finishing");
}

/*//}*/

}  // namespace ouster_nodelet

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(ouster_nodelet::OusterCloudNodelet, nodelet::Nodelet);
