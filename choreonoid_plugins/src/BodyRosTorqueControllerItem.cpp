/**
  @file
  @author
 */

#include "BodyRosTorqueControllerItem.h"

using namespace cnoid;

void BodyRosTorqueControllerItem::initialize(ExtensionManager* ext) { 
  static bool initialized = false;
  int argc = 0;
  char** argv;

  if (! ros::isInitialized()) {
    ros::init(argc, argv, "choreonoid");
  }

  if (! initialized) {
    ext->itemManager().registerClass<BodyRosTorqueControllerItem>("BodyRosTorqueControllerItem");
    ext->itemManager().addCreationPanel<BodyRosTorqueControllerItem>();
    initialized = true;
  }
}

BodyRosTorqueControllerItem::BodyRosTorqueControllerItem()
{
  controllerTarget         = 0;
  control_mode_name_       = "torque_control";
  has_trajectory_          = false;
  pdc_parameter_filename_  = "";
}

BodyRosTorqueControllerItem::BodyRosTorqueControllerItem(const BodyRosTorqueControllerItem& org)
  : BodyRosJointControllerItem(org)
{
  controllerTarget         = 0;
  control_mode_name_       = "torque_control";
  has_trajectory_          = false;
  pdc_parameter_filename_  = "";
}

BodyRosTorqueControllerItem::~BodyRosTorqueControllerItem()
{
  stop();
}

void BodyRosTorqueControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
  BodyRosJointControllerItem::doPutProperties(putProperty);

  putProperty("PD control parameter file", pdc_parameter_filename_, changeProperty(pdc_parameter_filename_));

  return;
}

Item* BodyRosTorqueControllerItem::doDuplicate() const
{
  return new BodyRosTorqueControllerItem(*this);
}

bool BodyRosTorqueControllerItem::store(Archive& archive)
{
  BodyRosJointControllerItem::store(archive);

  if (! pdc_parameter_filename_.empty()) {
    archive.writeRelocatablePath("pdcParameterFilename", pdc_parameter_filename_);
  }

  return true;
}

bool BodyRosTorqueControllerItem::restore(const Archive& archive)
{
  BodyRosJointControllerItem::restore(archive);

  archive.readRelocatablePath("pdcParameterFilename", pdc_parameter_filename_);

  return true;
}

bool BodyRosTorqueControllerItem::set_pdc_parameters(Listing* src, std::vector<double>& dst)
{
  if (! src) {
    return false;
  }

  for (size_t i = 0; i < src->size(); i++) {
    try {
      dst[i] = src->at(i)->toDouble();
    } catch(const ValueNode::NotScalarException ex) {
      MessageView::instance()->putln(
        MessageView::ERROR, boost::format("%1% (%1%)") % ex.message() % pdc_parameter_filename_
        );
      return false;
    }
  }

  return true;
}

bool BodyRosTorqueControllerItem::load_pdc_parameters()
{
  YAMLReader reader = YAMLReader();
  ValueNode* vnode;
  Mapping*   mapping;
  Listing*   listing;
  bool       result;

  pgain.resize(body()->numJoints());
  dgain.resize(body()->numJoints());
  u_lower.resize(body()->numJoints());
  u_upper.resize(body()->numJoints());

  if (pdc_parameter_filename_.empty()) {
    MessageView::instance()->putln(MessageView::ERROR, boost::format("'PD control parameter file' is empty"));
    return false;
  } else if (! reader.load(pdc_parameter_filename_)) {
    MessageView::instance()->putln(
      MessageView::ERROR, 
      boost::format("PD control parameter file load failed (%1%)") % pdc_parameter_filename_
      );
    return false;
  } else if (reader.numDocuments() != 1) {
    MessageView::instance()->putln(
      MessageView::ERROR,
      boost::format("invalid format found (%1%)") % pdc_parameter_filename_
      );
    return false;
  }

  vnode = reader.document(0);

  if (! vnode) {
    MessageView::instance()->putln(
      MessageView::ERROR,
      boost::format("file is empty (%1%)") % pdc_parameter_filename_
      );
    return false;
  } else if (vnode->nodeType() != ValueNode::MAPPING) {
    MessageView::instance()->putln(
      MessageView::ERROR,
      boost::format("invalid node type found (%1%)") % pdc_parameter_filename_
      );
    return false;
  }

  mapping = vnode->toMapping();

  if (mapping->empty() || mapping->size() != 4) {
    MessageView::instance()->putln(
      MessageView::ERROR,
      boost::format("mismatch of number of the parameters (%1%)") % pdc_parameter_filename_
      );
    return false;
  }

  for (Mapping::const_iterator it = mapping->begin(); it != mapping->end(); it++) {
    if (it->second->nodeType() != ValueNode::LISTING) {
      MessageView::instance()->putln(
        MessageView::ERROR,
        boost::format("invalid node type found (%1%)") % pdc_parameter_filename_
        );
      return false;
    }

    listing = it->second->toListing();

    if (listing->size() != body()->numJoints()) {
      MessageView::instance()->putln(
        MessageView::ERROR,
        boost::format("joint size mismatch (%1%: %2% joint: %3%)") % it->first % listing->size() % body()->numJoints()
        );
      return false;
    }

    if (it->first == "pgain") {
      result = set_pdc_parameters(listing, pgain);
    } else if (it->first == "dgain") {
      result = set_pdc_parameters(listing, dgain);
    } else if (it->first == "u_lower") {
      result = set_pdc_parameters(listing, u_lower);
    } else if (it->first == "u_upper") {
      result = set_pdc_parameters(listing, u_upper);
    } else {
      MessageView::instance()->putln(
        MessageView::ERROR,
        boost::format("invalid key found %1%") % it->first
        );
      return false;
    }

    if (! result) {
      return false;
    }
  }

  return true;
}

bool BodyRosTorqueControllerItem::hook_of_start()
{
#if (DEBUG_ROS_JOINT_CONTROLLER > 0)
  ROS_DEBUG("%s: Called.", __PRETTY_FUNCTION__);
#endif  /* DEBUG_ROS_JOINT_CONTROLLER */

  if (! load_pdc_parameters()) {
    return false;
  }

  qref_old_.resize(body()->numJoints());
  q_old_.resize(body()->numJoints());

  for (size_t i = 0; i < body()->numJoints(); i++) {
    qref_old_[i] = body()->joint(i)->q();
  }

  q_old_ = qref_old_;

  return true;
}

void BodyRosTorqueControllerItem::pd_control(Link* joint, double q_ref)
{
  double q;
  double dq_ref;
  double dq;
  double u;
  size_t i;

  if (! joint) {
    ROS_ERROR("%s: Invalid arguments", __PRETTY_FUNCTION__);
    return;
  }

  i = joint->jointId();
  q = joint->q();

  if (std::isnan(q_ref)) {
    ROS_ERROR("joint id %03d (%s): joint angle setting is NaN", joint->jointId(), joint->name().c_str());
    goto done;
  } else if (q_ref < joint->q_lower() || q_ref > joint->q_upper()) {
    ROS_ERROR("joint id %03d (%s): joint angle setting is over limit (lower %f upper %f set %f)",
              joint->jointId(), joint->name().c_str(), joint->q_lower(), joint->q_upper(), q_ref);
    goto done;
  } 

  dq_ref = (q_ref - qref_old_[i]) / timeStep_;

  if (std::isnan(dq_ref)) {
    ROS_ERROR("joint id %03d (%s): calculate dq_ref, result is NaN", joint->jointId(), joint->name().c_str());
    goto done;
  }

  dq = (q - q_old_[i]) / timeStep_;

  if (std::isnan(dq)) {
    ROS_ERROR("joint id %03d (%s): calculate dq, result is NaN", joint->jointId(), joint->name().c_str());
    goto done;
  }

  u = (q_ref - q) * pgain[i] + (dq_ref - dq) * dgain[i];

  if (! std::isnan(u)) {
    if (u < u_lower[i]) {
      ROS_DEBUG("joint id %03d (%s): calculate u, result is over lower limt. (adjust %f -> %f)",
                joint->jointId(), joint->name().c_str(), u, u_lower[i]);
      u = u_lower[i];
    } else if (u > u_upper[i]) {
      ROS_DEBUG("joint id %03d (%s): calculate u, result is over upper limt. (adjust %f -> %f)", 
                joint->jointId(), joint->name().c_str(), u, u_upper[i]);
      u = u_upper[i];
    }

#if (DEBUG_ROS_JOINT_CONTROLLER > 0)
    ROS_DEBUG("-- joint id %03d (%s) --", joint->jointId(), joint->name().c_str());
    ROS_DEBUG("time step %f", timeStep_);
    ROS_DEBUG("dq_lower %f dq_upper %f", joint->dq_lower(), joint->dq_upper());
    ROS_DEBUG("pgain %f dgain %f", pgain[i], dgain[i]);
    ROS_DEBUG("qref_old_ %f q_old_ %f", qref_old_[i], q_old_[i]);
    ROS_DEBUG("q_ref %f q %f dq_ref %f dq %f u %f", q_ref, q, dq_ref, dq, u);
    ROS_DEBUG("--");
#endif  /* DEBUG_ROS_JOINT_CONTROLLER */

    joint->u() = u;
    qref_old_[i] = q_ref;
  } else {
    ROS_ERROR("joint id %03d (%s): calculate u, result is NaN", joint->jointId(), joint->name().c_str());
  }

 done:
  q_old_[i] = q;

  return;
}

void BodyRosTorqueControllerItem::keep_attitude()
{
  for (size_t i = 0; i < body()->numJoints(); i++) {
    pd_control(body()->joint(i), qref_old_[i]);
  }

  return;
}

void BodyRosTorqueControllerItem::apply_message(Link* joint, size_t idx, trajectory_msgs::JointTrajectoryPoint* point)
{
  double u;

  if (! joint || ! point) {
    ROS_ERROR("%s: Invalid arguments", __PRETTY_FUNCTION__);
    return;
  }

  if (point->effort.size() < 1) {
    pd_control(joint, point->positions[idx]);
  } else {
    u = point->effort[idx];

    if (! std::isnan(u)) {
      joint->u() = u;
    } else {
      ROS_ERROR("Effort setting is NaN (%s)", joint->name().c_str());
    }
  }

  return;
}

bool BodyRosTorqueControllerItem::copy_message(
      const trajectory_msgs::JointTrajectoryPoint* msg,
      trajectory_msgs::JointTrajectoryPoint* dst,
      size_t idx,
      unsigned int jsz
      )
{
  bool has_effort;

  if (! msg || ! dst || jsz < 1) {
    ROS_ERROR("%s: Invalid arguments", __PRETTY_FUNCTION__);
    return false;
  }

  if (msg->velocities.size() > 0 || msg->accelerations.size() > 0) {
    ROS_WARN("Can not setting velocities and accelerations in this torque controller");
    return false;
  }

  dst->positions.resize(jsz);

  if (msg->effort.size() > 0) {
    dst->effort.resize(jsz);
    has_effort = true;
  } else {
    dst->effort.resize(0);
    has_effort = false;
  }

  for (size_t i = 0; i < jsz; i++) {
    dst->positions[i] = (i < msg->positions.size()) ? msg->positions[i] : 0.0;

    if (has_effort) {
      dst->effort[i] = (i < msg->effort.size()) ? msg->effort[i] : 0.0;
    }
  }

  dst->time_from_start = ros::Duration(msg->time_from_start.sec, msg->time_from_start.nsec);

  return true;
}

