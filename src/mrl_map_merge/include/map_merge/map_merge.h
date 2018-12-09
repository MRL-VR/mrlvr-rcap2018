/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Zhi Yan.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#ifndef MAP_MERGE_H_
#define MAP_MERGE_H_

#include <atomic>
#include <forward_list>
#include <mutex>
#include <unordered_map>

#include <combine_grids/merging_pipeline.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

namespace map_merge
{
struct MapSubscription {
  // protects consistency of writable_map and readonly_map
  // also protects reads and writes of shared_ptrs
  std::mutex mutex;
  std::string robot_name;
  geometry_msgs::Transform initial_pose;
  nav_msgs::OccupancyGrid::Ptr writable_map;
  nav_msgs::OccupancyGrid::ConstPtr readonly_map;
  nav_msgs::Odometry::ConstPtr readonly_odom;
  nav_msgs::Path robot_path;
  geometry_msgs::PoseStamped last_global_pos;
  visualization_msgs::Marker global_path_marker;

  geometry_msgs::PolygonStamped::ConstPtr readonly_footprint;
  visualization_msgs::MarkerArray marked_victims;

  ros::Subscriber map_sub;
  ros::Subscriber map_updates_sub;
  ros::Subscriber map_odom;
  ros::Subscriber map_footprint;
  ros::Subscriber victim_marker;


  // publishing
  ros::Publisher global_footprint_publisher_;
  ros::Publisher global_path_publisher_;
  ros::Publisher global_marker_publisher_;
  ros::Publisher marked_victims_publisher_;

};

class MapMerge
{
private:
  ros::NodeHandle node_;

  /* parameters */
  double merging_rate_;
  double discovery_rate_;
  double estimation_rate_;
  double confidence_threshold_;
  std::string robot_map_topic_;
  std::string robot_map_updates_topic_;
  std::string robot_map_odom_topic;
  std::string robot_footprint_topic_;
  std::string robot_namespace_;
  std::string world_frame_;
  bool have_initial_poses_;

  // publishing
  ros::Publisher merged_map_publisher_;
  ros::Subscriber save_map_subscriber_;

  // maps robots namespaces to maps. does not own
  std::unordered_map<std::string, MapSubscription*> robots_;
  // owns maps -- iterator safe
  std::forward_list<MapSubscription> subscriptions_;
  size_t subscriptions_size_;
  boost::shared_mutex subscriptions_mutex_;
  combine_grids::MergingPipeline pipeline_;
  std::mutex pipeline_mutex_;
  nav_msgs::OccupancyGridPtr last_merged_map;

  std::string robotNameFromTopic(const std::string& topic);
  bool isRobotMapTopic(const ros::master::TopicInfo& topic);
  bool getInitPose(const std::string& name, geometry_msgs::Transform& pose);

  void fullMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr& msg,
                     MapSubscription& map);

  void odomUpdate(const nav_msgs::Odometry::ConstPtr& msg,
                     MapSubscription& map);

  void victimUpdate(const std_msgs::Int32::ConstPtr& msg, MapSubscription& map);


  void footprintUpdate(const geometry_msgs::PolygonStamped::ConstPtr& msg,
                     MapSubscription& map);

  void partialMapUpdate(const map_msgs::OccupancyGridUpdate::ConstPtr& msg,
                        MapSubscription& map);

  void saveMapCallback(const std_msgs::String::ConstPtr& msg);
public:
  MapMerge();

  void spin();
  void executetopicSubscribing();
  void executemapMerging();
  void executeposeEstimation();

  void topicSubscribing();
  void mapMerging();
  /**
   * @brief Estimates initial positions of grids
   * @details Relevant only if initial poses are not known
   */
  void poseEstimation();
};

}  // namespace map_merge

#endif /* MAP_MERGE_H_ */
