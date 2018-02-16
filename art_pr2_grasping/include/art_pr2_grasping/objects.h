// Copyright 2016 Robo@FIT

#ifndef ART_PR2_GRASPING_OBJECTS_H
#define ART_PR2_GRASPING_OBJECTS_H

#include <string>
#include <set>
#include <map>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "art_msgs/InstancesArray.h"
#include "art_msgs/ObjInstance.h"
#include "art_msgs/ObjectType.h"
#include "art_msgs/getObjectType.h"
#include <boost/thread/recursive_mutex.hpp>
#include <moveit_visual_tools/visual_tools.h>
#include <stdexcept>
#include <vector>

namespace art_pr2_grasping
{
typedef struct
{
  std::string object_id;
  geometry_msgs::PoseStamped pose;
  art_msgs::ObjectType type;
  bool on_table;
}
TObjectInfo;

typedef std::map<std::string, TObjectInfo> TObjectMap;
typedef std::map<std::string, art_msgs::ObjectType> TObjCache;

class Objects
{
public:
  Objects(boost::shared_ptr<tf::TransformListener> tfl, std::string target_frame);

  bool isKnownObject(std::string id);

  bool isGrasped(std::string object_id)
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);
    return grasped_objects_.count(object_id);
  }

  TObjectInfo getObject(std::string object_id);  // TODO(ZdenekM) return pointer?

  std::vector<std::string> getObjects();

  void setGrasped(std::string object_id, bool grasped);

  void clear();

  void setPaused(bool paused, bool clear = false);

  void publishObject(std::string object_id);

  void setPose(std::string object_id, geometry_msgs::PoseStamped ps);

private:
  std::string objects_frame_id_;
  TObjectMap objects_;
  boost::recursive_mutex mutex_;

  boost::shared_ptr<tf::TransformListener> tfl_;
  ros::ServiceClient object_type_srv_;
  std::string target_frame_;
  ros::Subscriber obj_sub_;
  ros::NodeHandle nh_;

  bool transformPose(geometry_msgs::PoseStamped& ps);

  void detectedObjectsCallback(const art_msgs::InstancesArrayConstPtr& msg);

  moveit_visual_tools::VisualToolsPtr visual_tools_;

  std::set<std::string> grasped_objects_;

  bool paused_;

  TObjCache obj_cache_;
};

}  // namespace art_pr2_grasping

#endif  // ART_PR2_GRASPING_OBJECTS_H
