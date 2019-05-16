/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the Velodyne 3D LIDARs
 */

#ifndef _VELODYNE_DRIVER_H_
#define _VELODYNE_DRIVER_H_ 1

#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>

#include <velodyne_driver/input.h>
#include <velodyne_driver/VelodyneNodeConfig.h>
#include <cplus_xsens_driver/packet_counter.h>
#include <boost/thread.hpp>
#include <vector>



namespace velodyne_driver
{

class VelodyneDriver
{
public:

  VelodyneDriver(ros::NodeHandle node,
                 ros::NodeHandle private_nh);
  ~VelodyneDriver() {}

  bool poll(void);

  void PacketCallback (const cplus_xsens_driver::packet_counterConstPtr &packet);
  std::queue<cplus_xsens_driver::packet_counter> counter_time;
  double packet_counter_;
  double old_packet_counter_;
private:

  ///Callback for dynamic reconfigure
  void callback(velodyne_driver::VelodyneNodeConfig &config,
              uint32_t level);
  /// Callback for diagnostics update for lost communication with vlp
  void diagTimerCallback(const ros::TimerEvent&event);

  void pop_time_counter(void);
  ///Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<velodyne_driver::
              VelodyneNodeConfig> > srv_;

  // configuration parameters
  struct
  {
    std::string frame_id;            ///< tf frame ID
    std::string model;               ///< device model name
    int    npackets;                 ///< number of packets to collect
    double rpm;                      ///< device rotation rate (RPMs)
    int cut_angle;                   ///< cutting angle in 1/100°
    double time_offset;              ///< time in seconds added to each velodyne time stamp
  } config_;

  boost::shared_ptr<Input> input_;
  ros::Publisher output_;

  /** diagnostics updater */
  ros::Timer diag_timer_;
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;

  ros::Subscriber packet_sub_;   //Subscribe for packet counter
  boost::mutex read_write_mtx_;

  double synctime_sec_;
  uint64_t synctime_nsec_; 
  int hour_;

  uint32_t counter1_;
  uint32_t counter2_;
  ros::Time counter1_time_;
  ros::Time counter2_time_;

  ros::Time time2_;
};

} // namespace velodyne_driver

#endif // _VELODYNE_DRIVER_H_
