/**
 * @file
 * @brief Higher-level functions to read data from the ouster sensors as ROS
 * messages
 */

#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <chrono>
#include <string>

#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/point.h"
#include <tf/transform_listener.h>

namespace ouster_ros {

namespace sensor = ouster::sensor;
using Cloud = pcl::PointCloud<Point>;
using ns = std::chrono::nanoseconds;

/**
 * Read an imu packet into a ROS message. Blocks for up to a second if no data
 * is available.
 * @param cli the sensor client
 * @param pm the destination packet message
 * @return whether reading was successful
 */
bool read_imu_packet(const sensor::client& cli, PacketMsg& pm,
                     const sensor::packet_format& pf);

/**
 * Read a lidar packet into a ROS message. Blocks for up to a second if no data
 * is available.
 * @param cli the sensor client
 * @param pm the destination packet message
 * @return whether reading was successful
 */
bool read_lidar_packet(const sensor::client& cli, PacketMsg& pm,
                       const sensor::packet_format& pf);

/**
 * Parse an imu packet message into a ROS imu message
 * @param pm packet message populated by read_imu_packet
 * @param frame the frame to set in the resulting ROS message
 * @return ROS sensor message with fields populated from the packet
 */
sensor_msgs::Imu packet_to_imu_msg(const PacketMsg& pm,
                                   const std::string& frame,
                                   const sensor::packet_format& pf);

/**
 * Populate a PCL point cloud from a LidarScan
 * @param xyz_lut lookup table from sensor beam angles (see lidar_scan.h)
 * @param scan_ts scan start used to caluclate relative timestamps for points
 * @param ls input lidar data
 * @param cloud output pcl pointcloud to populate
 */
void scan_to_cloud(const ouster::XYZLut& xyz_lut,
                   ouster::LidarScan::ts_t scan_ts, const ouster::LidarScan& ls,
                   ouster_ros::Cloud& cloud);

/**
 * Populate a PCL point cloud from a LidarScan taking into account
 * the movement of the sensor frame compared to a fixed frame. It is
 * useful to correctly transform each point of the lidar based on
 * their timestamp when the lidar is moving.
 * @param xyz_lut lookup table from sensor beam angles (see lidar_scan.h)
 * @param scan_ts scan start used to caluclate relative timestamps for points
 * @param ls input lidar data
 * @param cloud output pcl pointcloud to populate
 * @param listener TF listener used to interpolate the transform between fixed_frame and sensor_frame
 * @param fixed_frame the fixed frame
 * @param sensor_frame the sensor frame
 * @param waitForTransform maximum time to wait to get TF between sensor_frame and fixed_frame accordingly to start and end stamps of the LidarScan (set 0 do disable, but some points may not be corrected if some TFs are not yet received in time)
 */

void scan_to_cloud(const ouster::XYZLut& xyz_lut,
                   ouster::LidarScan::ts_t scan_ts, const ouster::LidarScan& ls,
                   ouster_ros::Cloud& cloud,
                   std::shared_ptr<tf::TransformListener> listener,
                   const std::string & fixed_frame,
                   const std::string & sensor_frame,
                   const double & waitForTransform = 0.01);
/**
 * Serialize a PCL point cloud to a ROS message
 * @param cloud the PCL point cloud to convert
 * @param timestamp the timestamp to give the resulting ROS message
 * @param frame the frame to set in the resulting ROS message
 * @return a ROS message containing the point cloud
 */
sensor_msgs::PointCloud2 cloud_to_cloud_msg(const Cloud& cloud, ns timestamp,
                                            const std::string& frame);

/**
 * Convert transformation matrix return by sensor to ROS transform
 * @param mat transformation matrix return by sensor
 * @param frame the parent frame of the published transform
 * @param child_frame the child frame of the published transform
 * @return ROS message suitable for publishing as a transform
 */
geometry_msgs::TransformStamped transform_to_tf_msg(
    const ouster::mat4d& mat, const std::string& frame,
    const std::string& child_frame);
}  // namespace ouster_ros
