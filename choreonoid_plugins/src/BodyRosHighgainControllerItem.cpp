/**
  @file BodyRosHighgainControllerItem.cpp
  @author
 */

#include "BodyRosHighgainControllerItem.h"

using namespace cnoid;

void BodyRosHighgainControllerItem::initialize(ExtensionManager* ext) { 
  static bool initialized = false;
  int argc = 0;
  char** argv;

  if (! ros::isInitialized()) {
    ros::init(argc, argv, "choreonoid");
  }

  if (! initialized) {
    ext->itemManager().registerClass<BodyRosHighgainControllerItem>("BodyRosHighgainControllerItem");
    ext->itemManager().addCreationPanel<BodyRosHighgainControllerItem>();
    initialized = true;
  }
}

BodyRosHighgainControllerItem::BodyRosHighgainControllerItem()
{
  controllerTarget         = 0;
  control_mode_name_       = "highgain_control";
  has_trajectory_          = false;
}

BodyRosHighgainControllerItem::BodyRosHighgainControllerItem(const BodyRosHighgainControllerItem& org)
  : BodyRosJointControllerItem(org)
{
  controllerTarget         = 0;
  control_mode_name_       = "highgain_control";
  has_trajectory_          = false;
}

BodyRosHighgainControllerItem::~BodyRosHighgainControllerItem()
{
  stop();
}

Item* BodyRosHighgainControllerItem::doDuplicate() const
{
  return new BodyRosHighgainControllerItem(*this);
}

bool BodyRosHighgainControllerItem::hook_of_start()
{
  hg_calculated_.resize(body()->numJoints());
  qref_.resize(body()->numJoints());

  for (size_t i = 0; i < body()->numJoints(); i++) {
    hg_calculated_[i] = false;
    qref_[i]          = body()->joint(i)->q();
  }

  return true;
}

void BodyRosHighgainControllerItem::keep_attitude()
{
  for (size_t i = 0; i < body()->numJoints(); i++) {
    if (! hg_calculated_[i]) {
      calculate_hg_parameter(body()->joint(i), qref_[i], 0, 0);
    } else {
      hg_calculated_[i] = false;
    }
  }

  return;
}

void BodyRosHighgainControllerItem::calculate_hg_parameter(
      Link* joint,
      double qref,
      double* velocity,
      double* acceleration
      )
{
  double dq;
  double ddq;
  int i;

  if (! joint) {
    ROS_ERROR("%s: Invalid arguments", __PRETTY_FUNCTION__);
    return;
  }

  i = joint->jointId();

  if (std::isnan(qref)) {
    ROS_ERROR("joint id %03d (%s): joint angle setting is NaN", i, joint->name().c_str());
    return;
  } else if (qref < joint->q_lower() || qref > joint->q_upper()) {
    ROS_ERROR("joint id %03d (%s): joint angle setting is over limit (lower %f upper %f set %f)",
              i, joint->name().c_str(), joint->q_lower(), joint->q_upper(), qref);
    return;
  }

  if (velocity) {
    dq = *velocity;
  } else {
    dq = (qref - joint->q()) / timeStep_;
  }

  if (std::isnan(dq)) {
    ROS_ERROR("joint id %03d (%s): calculate joint velocity, result is NaN", i, joint->name().c_str());
    return;
  }

  if (acceleration) {
    ddq = *acceleration;
  } else {
    ddq = dq / timeStep_;
  }

  if (std::isnan(ddq)) {
    ROS_ERROR("joint id %03d (%s): calculate joint acceleration, result is NaN", i, joint->name().c_str());
    return;
  }

  ROS_DEBUG("joint id %03d (%s): q %f dq %f ddq %f q (current) %f",
            i, joint->name().c_str(), qref, dq, ddq, joint->q());

  joint->q()   = qref;
  joint->dq()  = dq;
  joint->ddq() = ddq;
  qref_[i]     = qref;

  return;
}

void BodyRosHighgainControllerItem::apply_message(Link* joint, size_t idx, trajectory_msgs::JointTrajectoryPoint* point)
{
  if (! joint || ! point) {
    ROS_ERROR("%s: Invalid arguments", __PRETTY_FUNCTION__);
    return;
  }

  hg_calculated_[ joint->jointId() ] = true;

  calculate_hg_parameter(
    joint,
    point->positions[idx],
    (point->velocities.size() > 0) ? &point->velocities[idx] : 0,
    (point->accelerations.size() > 0) ? &point->accelerations[idx] : 0
    );

  return;
}

bool BodyRosHighgainControllerItem::copy_message(
      const trajectory_msgs::JointTrajectoryPoint* msg,
      trajectory_msgs::JointTrajectoryPoint* dst,
      size_t idx,
      unsigned int jsz
      )
{
  bool has_vel;
  bool has_accel;

  if (! msg || ! dst || jsz < 1) {
    ROS_ERROR("%s: Invalid arguments", __PRETTY_FUNCTION__);
    return false;
  }

  if (msg->effort.size() > 0) {
    ROS_WARN("Can not setting effort in this high-gain controller");
    return false;
  }

  dst->positions.resize(jsz);

  if (msg->velocities.size() > 0) {
    dst->velocities.resize(jsz);
    has_vel = true;
  } else {
    dst->velocities.resize(0);
    has_vel = false;
  }

  if (msg->accelerations.size() > 0) {
    dst->accelerations.resize(jsz);
    has_accel = true;
  } else {
    dst->accelerations.resize(0);
    has_accel = false;
  }

  for (size_t i = 0; i < jsz; i++) {
    dst->positions[i] = (i < msg->positions.size()) ? msg->positions[i] : 0.0;

    if (has_vel) {
      dst->velocities[i] = (i < msg->velocities.size()) ? msg->velocities[i] : 0.0;
    }

    if (has_accel) {
      dst->accelerations[i] = (i < msg->accelerations.size()) ? msg->accelerations[i] : 0.0;
    }
  }

  dst->time_from_start = ros::Duration(msg->time_from_start.sec, msg->time_from_start.nsec);

  return true;
}

