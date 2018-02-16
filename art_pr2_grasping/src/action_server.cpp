// Copyright 2016 Robo@FIT

#include "art_pr2_grasping/action_server.h"
#include <string>

namespace art_pr2_grasping
{
artActionServer::artActionServer(boost::shared_ptr<tf::TransformListener> tfl, boost::shared_ptr<Objects> objects,
                                 std::string group_name, std::string default_target, std::string gripper_state_topic)
  : max_attempts_(3), artPr2Grasping(tfl, objects, group_name, default_target, gripper_state_topic)
{
  as_.reset(new actionlib::SimpleActionServer<art_msgs::PickPlaceAction>(
              nh_, "/art/robot/" + group_name_ + "/pp", boost::bind(&artActionServer::executeCB, this, _1), false));
}

bool artActionServer::init()
{
  while (!tfl_->waitForTransform(getPlanningFrame(), "marker", ros::Time(0), ros::Duration(5.0)))
  {
    ROS_INFO_STREAM_NAMED(group_name_, "Waiting for transform...");
  }

  if (getGripperValue() == 1000)
  {
    // TODO(ZdenekM): exception / NaN
    ROS_ERROR_NAMED(group_name_, "Can't get gripper state");
    return false;
  }

  if (!addTable("marker"))
  {
    ROS_ERROR("failed to add table");
    return false;
  }

  as_->start();
  return true;
}

void artActionServer::executeCB(const art_msgs::PickPlaceGoalConstPtr& goal)
{
  art_msgs::PickPlaceResult res;
  art_msgs::PickPlaceFeedback f;

  ROS_INFO_STREAM_NAMED(group_name_, "Got goal, operation: " << goal->operation);

  // TODO(ZdenekM): hack - table sometimes gets deleted from planning scene
  addTable("marker");

  // TODO(ZdenekM): check /art/pr2/xyz_arm/interaction/state topic?

  switch (goal->operation)
  {
  case art_msgs::PickPlaceGoal::RESET:
  {
    ROS_INFO_NAMED(group_name_, "RESET");
    objects_->setPaused(false);
    move_group_->detachObject();
    grasped_object_.reset();
    objects_->clear();
    // publishObject(); // WTF?

    res.result = art_msgs::PickPlaceResult::SUCCESS;
    as_->setSucceeded(res);
  }
  break;

  case art_msgs::PickPlaceGoal::GET_READY:
  {
    if (!getReady())
    {
      res.result = art_msgs::PickPlaceResult::FAILURE;
      as_->setAborted(res, "failed to get ready");
      return;
    }

    res.result = art_msgs::PickPlaceResult::SUCCESS;
    as_->setSucceeded(res);
  }
  break;

  case art_msgs::PickPlaceGoal::PICK_FROM_FEEDER:
  case art_msgs::PickPlaceGoal::PICK_OBJECT_ID:
  {
    if (!objects_->isKnownObject(goal->object))
    {
      res.result = art_msgs::PickPlaceResult::BAD_REQUEST;
      as_->setAborted(res, "unknown object id");
      return;
    }

    objects_->setGrasped(goal->object, true);  // will stop publishing it to collision scene

    int tries = max_attempts_;

    bool grasped = false;

    dont_try_again_ = false;
    while (tries > 0 && !as_->isPreemptRequested() && !dont_try_again_)
    {
      f.attempt = (max_attempts_ - tries) + 1;
      ROS_INFO_NAMED(group_name_, "Pick %d", f.attempt);
      tries--;
      as_->publishFeedback(f);

      grasped = pick(goal->object, goal->operation == art_msgs::PickPlaceGoal::PICK_FROM_FEEDER);  // todo flag if it
      // make sense to
      // try again

      objects_->setPaused(false);

      // (type of failure)
      if (grasped)
        break;

      objects_->publishObject(goal->object);
    }

    if (as_->isPreemptRequested())
    {
      as_->setPreempted(res, "pick cancelled");
      objects_->setGrasped(goal->object, false);
      return;
    }

    if (!grasped)
    {
      res.result = art_msgs::PickPlaceResult::FAILURE;
      as_->setAborted(res, "pick failed");
      objects_->setGrasped(goal->object, false);
      return;
    }

    res.result = art_msgs::PickPlaceResult::SUCCESS;
    as_->setSucceeded(res);
  }
  break;

  case art_msgs::PickPlaceGoal::PLACE_TO_POSE:
  {
    geometry_msgs::PoseStamped p2 = goal->pose;  // place goal

    // do transformation in advance - before timestamp get too old
    if (!transformPose(p2))
    {
      res.result = art_msgs::PickPlaceResult::FAILURE;
      as_->setAborted(res, "failed to transform pose");
      return;
    }

    if (!hasGraspedObject())
    {
      ROS_ERROR_NAMED(group_name_, "No object grasped - can't do PLACE.");
      res.result = art_msgs::PickPlaceResult::FAILURE;
      as_->setAborted(res, "No object grasped!");
      return;
    }

    int tries = max_attempts_;

    bool placed = false;

    dont_try_again_ = false;

    while (tries > 0 && !as_->isPreemptRequested() && !dont_try_again_)
    {
      f.attempt = (max_attempts_ - tries) + 1;
      ROS_INFO_NAMED(group_name_, "Place %d", f.attempt);
      tries--;
      as_->publishFeedback(f);
      placed = place(p2.pose, goal->z_axis_angle_increment, goal->keep_orientation);
      if (placed)
        break;
    }

    if (as_->isPreemptRequested())
    {
      as_->setPreempted(res, "place cancelled");
      return;
    }

    if (!placed)
    {
      ROS_ERROR_NAMED(group_name_, "Unable to place the object.");
      res.result = art_msgs::PickPlaceResult::FAILURE;
      as_->setAborted(res, "place failed");
      return;
    }

    objects_->setGrasped(goal->object, false);
    res.result = art_msgs::PickPlaceResult::SUCCESS;
    as_->setSucceeded(res);
    ROS_INFO_NAMED(group_name_, "Placed");
  }
  break;

  default:

    res.result = art_msgs::PickPlaceResult::BAD_REQUEST;
    as_->setAborted(res, "Unknown operation");
  }
}

}  // namespace art_pr2_grasping
