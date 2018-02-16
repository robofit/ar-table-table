// Copyright 2016 Robo@FIT

#include "art_pr2_grasping/node.h"
#include <string>
#include <vector>

namespace art_pr2_grasping
{
GraspingNode::GraspingNode() : target_frame_("odom_combined"), nh_("~")
{
  tfl_.reset(new tf::TransformListener());
  objects_.reset(new Objects(tfl_, target_frame_));

  std::vector<std::string> groups;
  std::vector<std::string> gripper_state_topics;
  std::vector<std::string> default_poses;

  nh_.getParam("groups", groups);
  nh_.getParam("gripper_state_topics", gripper_state_topics);
  nh_.getParam("default_poses", default_poses);

  ROS_ASSERT(groups.size() == gripper_state_topics.size() && groups.size() == default_poses.size() &&
             groups.size() > 0);

  for (int i = 0; i < groups.size(); i++)
  {
    action_servers_.push_back(boost::shared_ptr<artActionServer>(
                                new artActionServer(tfl_,
                                                    objects_,
                                                    groups[i],
                                                    default_poses[i],
                                                    gripper_state_topics[i])));
  }

  // TODO(ZdenekM): visual_tools + callback from objects + publish/remove
  // collision objects
}

bool GraspingNode::init()
{
  for (int i = 0; i < action_servers_.size(); i++)
  {
    if (!action_servers_[i]->init())
      return false;
  }

  return true;
}

}  // namespace art_pr2_grasping

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_pick_place");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ROS_INFO("Starting...");

  art_pr2_grasping::GraspingNode node;
  ros::Duration(1).sleep();
  if (!node.init())
  {
    ROS_ERROR("Init failed.");
    return 1;
  }

  ROS_INFO("Started - waiting for goals.");

  ros::waitForShutdown();
  return 0;
}
