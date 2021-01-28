/**
 * @file
 * @brief Example node to visualize range, ambient and intensity images
 *
 * Publishes ~/range_image, ~/ambient_image, and ~/intensity_image.  Please bear
 * in mind that there is rounding/clamping to display 8 bit images. For computer
 * vision applications, use higher bit depth values in /os_cloud_node/points
 */

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ouster/autoexposure.h"
#include "ouster/beam_uniformity.h"
#include "ouster/client.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/ros.h"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace sensor = ouster::sensor;
namespace viz    = ouster::viz;

namespace ouster_nodelet
{

using pixel_type                  = uint8_t;
constexpr size_t bit_depth        = 8 * sizeof(pixel_type);
const size_t     pixel_value_max  = std::numeric_limits<pixel_type>::max();
constexpr double range_multiplier = 1.0 / 200.0;  // assuming 200 m range typical

/* class OusterImgNodelet //{ */

class OusterImgNodelet : public nodelet::Nodelet {

public:
  virtual void onInit();
  virtual ~OusterImgNodelet();

private:
  int         run();
  std::thread init_thread_;
  bool        is_initialized_ = false;

  ros::Subscriber pc_sub_;
  ros::Publisher  range_image_pub_;
  ros::Publisher  ambient_image_pub_;
  ros::Publisher  intensity_image_pub_;

  sensor::sensor_info info_;
  uint32_t            H_;
  uint32_t            W_;

  viz::AutoExposure            ambient_ae_, intensity_ae_;
  viz::BeamUniformityCorrector ambient_buc_;

  std::string encoding_;
};

//}

/* run() */ /*//{*/

int OusterImgNodelet::run() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  ROS_INFO("[OusterImgNodelet]: Initializing.");

  ouster_ros::OSConfigSrv cfg{};
  auto                    client = nh.serviceClient<ouster_ros::OSConfigSrv>("os_config");
  client.waitForExistence();
  if (!client.call(cfg)) {
    ROS_ERROR("[OusterImgNodelet]: Calling os config service failed");
    return false;
  }

  info_ = sensor::parse_metadata(cfg.response.metadata);
  H_    = info_.format.pixels_per_column;
  W_    = info_.format.columns_per_frame;

  /* const auto& px_offset = info_.format.pixel_shift_by_row; */

  range_image_pub_     = nh.advertise<sensor_msgs::Image>("range_image", 100);
  ambient_image_pub_   = nh.advertise<sensor_msgs::Image>("ambient_image", 100);
  intensity_image_pub_ = nh.advertise<sensor_msgs::Image>("intensity_image", 100);

  /* ouster_ros::Cloud cloud{}; */

  std::stringstream encoding_ss;
  encoding_ss << "mono" << bit_depth;
  encoding_ = encoding_ss.str();

  auto cloud_handler = [&](const sensor_msgs::PointCloud2::ConstPtr& m) {
    if (!is_initialized_) {
      ROS_INFO_THROTTLE(1.0, "Not yet initialized.");
      return;
    }
    ouster_ros::Cloud cloud{};
    pcl::fromROSMsg(*m, cloud);


    sensor_msgs::Image range_image;
    sensor_msgs::Image ambient_image;
    sensor_msgs::Image intensity_image;

    range_image.width    = W_;
    range_image.height   = H_;
    range_image.step     = W_;
    range_image.encoding = encoding_;
    range_image.data.resize(W_ * H_ * bit_depth / (8 * sizeof(*range_image.data.data())));
    range_image.header.stamp = m->header.stamp;

    ambient_image.width    = W_;
    ambient_image.height   = H_;
    ambient_image.step     = W_;
    ambient_image.encoding = encoding_;
    ambient_image.data.resize(W_ * H_ * bit_depth / (8 * sizeof(*ambient_image.data.data())));
    ambient_image.header.stamp = m->header.stamp;

    intensity_image.width    = W_;
    intensity_image.height   = H_;
    intensity_image.step     = W_;
    intensity_image.encoding = encoding_;
    intensity_image.data.resize(W_ * H_ * bit_depth / (8 * sizeof(*intensity_image.data.data())));
    intensity_image.header.stamp = m->header.stamp;

    using im_t = Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    im_t ambient_image_eigen(H_, W_);
    im_t intensity_image_eigen(H_, W_);


    for (size_t u = 0; u < H_; u++) {
      for (size_t v = 0; v < W_; v++) {
        /* const size_t vv    = (v + W_ - px_offset[u]) % W_; */
        const size_t vv = (v + W_ - info_.format.pixel_shift_by_row[u]) % W_;
        const size_t index = u * W_ + vv;
        const auto& pt = cloud[index];

        if (pt.range == 0) {
          reinterpret_cast<pixel_type*>(range_image.data.data())[u * W_ + v] = 0;
        } else {
          reinterpret_cast<pixel_type*>(range_image.data.data())[u * W_ + v] =
              pixel_value_max - std::min(std::round(pt.range * range_multiplier), static_cast<double>(pixel_value_max));
        }
        ambient_image_eigen(u, v) = pt.ambient;
        intensity_image_eigen(u, v) = pt.intensity;
      }
    }

    ambient_buc_.correct(ambient_image_eigen);
    ambient_ae_(Eigen::Map<Eigen::ArrayXd>(ambient_image_eigen.data(), W_ * H_));
    intensity_ae_(Eigen::Map<Eigen::ArrayXd>(intensity_image_eigen.data(), W_ * H_));
    ambient_image_eigen   = ambient_image_eigen.sqrt();
    intensity_image_eigen = intensity_image_eigen.sqrt();
    for (size_t u = 0; u < H_; u++) {
      for (size_t v = 0; v < W_; v++) {
        reinterpret_cast<pixel_type*>(ambient_image.data.data())[u * W_ + v]   = ambient_image_eigen(u, v) * pixel_value_max;
        reinterpret_cast<pixel_type*>(intensity_image.data.data())[u * W_ + v] = intensity_image_eigen(u, v) * pixel_value_max;
      }
    }

    range_image_pub_.publish(range_image);
    ambient_image_pub_.publish(ambient_image);
    intensity_image_pub_.publish(intensity_image);
    ROS_INFO_THROTTLE(3.0, "[OusterImgNodelet]: publishing images");
  };

  pc_sub_ = nh.subscribe<sensor_msgs::PointCloud2, const sensor_msgs::PointCloud2ConstPtr&>("points", 500, cloud_handler);

  is_initialized_ = true;

  ROS_INFO("[OusterImgNodelet]: Initialized.");

  return true;
}
/*//}*/

/* onInit //{ */

void OusterImgNodelet::onInit() {

  init_thread_ = std::thread(&OusterImgNodelet::run, this);
}

//}

/* ~OusterImgNodelet */ /*//{*/

OusterImgNodelet::~OusterImgNodelet() {
  init_thread_.join();
}

/*//}*/

}  // namespace ouster_nodelet

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(ouster_nodelet::OusterImgNodelet, nodelet::Nodelet);
