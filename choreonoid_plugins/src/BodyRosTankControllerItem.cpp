/**
  @file
  @author Shuuji Kajita
 */

#include "BodyRosTankControllerItem.h"

using namespace cnoid;

void BodyRosTankControllerItem::initialize(ExtensionManager* ext) { 
  static bool initialized = false;
  int argc = 0;
  char** argv;

  if (! ros::isInitialized()) {
    ros::init(argc, argv, "choreonoid");
  }

  if (! initialized) {
    ext->itemManager().registerClass<BodyRosTankControllerItem>("BodyRosTankControllerItem");
    ext->itemManager().addCreationPanel<BodyRosTankControllerItem>();
    initialized = true;
  }
}

BodyRosTankControllerItem::BodyRosTankControllerItem()
  : os(MessageView::instance()->cout())
{
  controllerTarget        = 0;
  pdc_parameter_filename_ = "";
}

BodyRosTankControllerItem::BodyRosTankControllerItem(const BodyRosTankControllerItem& org)
  : ControllerItem(org),
    os(MessageView::instance()->cout())
{
  controllerTarget        = 0;
  pdc_parameter_filename_ = "";
}

BodyRosTankControllerItem::~BodyRosTankControllerItem()
{
  stop();
}

//--------------------------------------------------------------------------
Item* BodyRosTankControllerItem::doDuplicate() const
{
  return new BodyRosTankControllerItem(*this);
}

void BodyRosTankControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
  putProperty("PD control parameter file", pdc_parameter_filename_, changeProperty(pdc_parameter_filename_));

  return;
}

bool BodyRosTankControllerItem::store(Archive& archive)
{
  if (! pdc_parameter_filename_.empty()) {
    archive.writeRelocatablePath("pdcParameterFilename", pdc_parameter_filename_);
  }

  return true;
}

bool BodyRosTankControllerItem::restore(const Archive& archive)
{
  archive.readRelocatablePath("pdcParameterFilename", pdc_parameter_filename_);

  return true;
}

//--------------------------------------------------------------------------
bool BodyRosTankControllerItem::start(Target* target)
{
  if (! target) {
    MessageView::instance()->putln(MessageView::ERROR, boost::format("Target not found"));
    return false;
  } else if (! target->body()) {
    MessageView::instance()->putln(MessageView::ERROR, boost::format("BodyItem not found"));
    return false;
  }

  controllerTarget = target;
  simulationBody   = target->body();
  timeStep_        = target->worldTimeStep();
  controlTime_     = target->currentTime();

  crawlerL = body()->link("CRAWLER_TRACK_L");
  crawlerR = body()->link("CRAWLER_TRACK_R");

  light = body()->findDevice<SpotLight>("MainLight");
  if(!light){
    MessageView::instance()->putln(MessageView::ERROR, boost::format("MainLight was not found"));
    return false;
  }

  std::string name = body()->name();
  std::replace(name.begin(), name.end(), '-', '_');

  if (hook_of_start() == false) {
    return false;
  }

  rosnode_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(name));
  cmd_track_speed = rosnode_->subscribe("track_speed", 1, 
					&BodyRosTankControllerItem::receive_message, this);
  cmd_cannon_pitch = rosnode_->subscribe("cannon_pitch", 1, 
					&BodyRosTankControllerItem::receive_cannon_pitch, this);
  cmd_cannon_yaw = rosnode_->subscribe("cannon_yaw", 1, 
					&BodyRosTankControllerItem::receive_cannon_yaw, this);
  cmd_light_onoff = rosnode_->subscribe("light_onoff", 1, 
					&BodyRosTankControllerItem::receive_light_onoff, this);

  if (hook_of_start_at_after_creation_rosnode() == false) {
    return false;
  }

  async_ros_spin_.reset(new ros::AsyncSpinner(0));
  async_ros_spin_->start();

  return true;
}

bool BodyRosTankControllerItem::hook_of_start()
{
#if (DEBUG_ROS_TANK_CONTROLLER > 0)
  ROS_DEBUG("%s: Called.", __PRETTY_FUNCTION__);
#endif  /* DEBUG_ROS_TANK_CONTROLLER */

  if (! load_pdc_parameters()) {
    return false;
  }

  lin_vel << 0, 0, 0;
  ang_vel << 0, 0, 0;

  cannon_pitch = 0.0;
  cannon_yaw   = 0.0;

  qref.resize(body()->numJoints());
  q_old_.resize(body()->numJoints());

  for (size_t i = 0; i < body()->numJoints(); i++) {
    qref[i] = body()->joint(i)->q();
  }

  q_old_ = qref;

  return true;
}

bool BodyRosTankControllerItem::hook_of_start_at_after_creation_rosnode()
{
#if (DEBUG_ROS_TANK_CONTROLLER > 0)
  ROS_DEBUG("%s: Called", __PRETTY_FUNCTION__);
#endif  /* DEBUG_ROS_TANK_CONTROLLER */

  light->on(false);   // turn off the light at start
  light->notifyStateChange();

  return true;
}

bool BodyRosTankControllerItem::set_pdc_parameters(Listing* src, std::vector<double>& dst)
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

bool BodyRosTankControllerItem::load_pdc_parameters()
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

  std::ostringstream os;
  os << "load_pdc_parameters(): body()->numJoints()=" << body()->numJoints();
  MessageView::instance()->putln(os.str());

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

void BodyRosTankControllerItem::receive_message(const geometry_msgs::Twist &twist)
{
  std::ostringstream os;
  os << "twist.linear.x= " << twist.linear.x << ", angular.z= " << twist.angular.z;
  MessageView::instance()->putln(os.str());

  lin_vel << twist.linear.x,  twist.linear.y,  twist.linear.z;
  ang_vel << twist.angular.x, twist.angular.y, twist.angular.z;

  return;
}

void BodyRosTankControllerItem::receive_cannon_pitch(const std_msgs::Float32 &pitch)
{
  std::ostringstream os;
  os << "(cannon) pitch= " << pitch.data;
  MessageView::instance()->putln(os.str());

  cannon_pitch = (double)pitch.data;

  return;
}

void BodyRosTankControllerItem::receive_cannon_yaw(const std_msgs::Float32 &yaw)
{
  std::ostringstream os;
  os << "(cannon) yaw= " << yaw.data;
  MessageView::instance()->putln(os.str());

  cannon_yaw = (double)yaw.data;

  return;
}

void BodyRosTankControllerItem::receive_light_onoff(const std_msgs::Bool &onoff)
{
  std::ostringstream os;
  os << "(cannon) light_onoff= " << onoff.data;
  MessageView::instance()->putln(os.str());

  light->on(onoff.data);
  light->notifyStateChange();

  return;
}

bool BodyRosTankControllerItem::control()
{
  controlTime_ = controllerTarget->currentTime();

  /* control cannon */
  qref[2] = -cannon_yaw;
  qref[3] = cannon_pitch;

  for (size_t i = 2; i < body()->numJoints(); i++) {
    pd_control(body()->joint(i), qref[i]);
  }

  /* control crawler */
  crawlerL->dq() = 2.0*(lin_vel(0) - ang_vel(2));
  crawlerR->dq() = 2.0*(lin_vel(0) + ang_vel(2));

  double DecayRate;
  //DecayRate = 0.997;  // original
  DecayRate = 0.99;

  lin_vel(0) = DecayRate*lin_vel(0);
  ang_vel(2) = DecayRate*ang_vel(2);

  return true;
}


void BodyRosTankControllerItem::pd_control(Link* joint, double q_ref)
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

  //dq_ref = (q_ref - qref_old_[i]) / timeStep_;
  dq_ref = 0.0;

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

#if (DEBUG_ROS_TANK_CONTROLLER > 0)
    ROS_DEBUG("-- joint id %03d (%s) --", joint->jointId(), joint->name().c_str());
    ROS_DEBUG("time step %f", timeStep_);
    ROS_DEBUG("dq_lower %f dq_upper %f", joint->dq_lower(), joint->dq_upper());
    ROS_DEBUG("pgain %f dgain %f", pgain[i], dgain[i]);
    ROS_DEBUG("qref %f q_old_ %f", qref[i], q_old_[i]);
    ROS_DEBUG("q_ref %f q %f dq_ref %f dq %f u %f", q_ref, q, dq_ref, dq, u);
    ROS_DEBUG("--");
#endif  /* DEBUG_ROS_TANK_CONTROLLER */

    joint->u() = u;
    //qref_old_[i] = q_ref;
  } else {
    ROS_ERROR("joint id %03d (%s): calculate u, result is NaN", 
	      joint->jointId(), joint->name().c_str());
  }

 done:
  q_old_[i] = q;

  return;
}
