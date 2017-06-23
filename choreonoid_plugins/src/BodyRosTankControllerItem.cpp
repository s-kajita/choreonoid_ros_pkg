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

  crawlerL = body()->link("TRACK_L");
  crawlerR = body()->link("TRACK_R");

  ballistic_z = body()->link("BALLISTIC_Z");
  ballistic_y = body()->link("BALLISTIC_Y");
  
  cannonball = body()->link("CANNONBALL");
  
  light = body()->findDevice<SpotLight>("Light");
  if(!light){
    MessageView::instance()->putln(MessageView::ERROR, boost::format("Light was not found"));
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
  cmd_fire = rosnode_->subscribe("fire", 1, 
					&BodyRosTankControllerItem::receive_fire, this);

  cannon_fired = false;
  
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

  double jsize = body()->numJoints();
  
  pgain.resize(jsize);
  dgain.resize(jsize);
  u_lower.resize(jsize);
  u_upper.resize(jsize);

  for(int j=0; j < jsize; j++){
    pgain[j] = 0;
    dgain[j] = 0;
    u_lower[j] = 0;
    u_upper[j] = 0;
  }

  pgain[0] = 30000.0;
  dgain[0] = 5000.0;
  u_lower[0] = -20.0;
  u_upper[0] =  20.0;

  pgain[1] = 30000.0;
  dgain[1] = 5000.0;
  u_lower[1] = -20.0;
  u_upper[1] =  20.0;
  
  // for stabile simulation of the cannonball dynamics
  for(int j=2; j<jsize; j++){
    dgain[j]= 10.0;
  }

  
#if 0
  if (! load_pdc_parameters()) {
    return false;
  }
#endif
  
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

void BodyRosTankControllerItem::receive_fire(const std_msgs::Bool &fire)
{
  std::ostringstream os;
  os << "(cannon) fire= " << fire.data;
  MessageView::instance()->putln(os.str());

  if(fire.data){
    cannon_fired = true;
    ballistic_z->q() = 0.0;
    cannonball->dq() = 100.0;
    cannonball->u()  = 0;
  }
  else {
    cannon_fired = false;
    ballistic_z->q() = 0.0;
    ballistic_y->q() = 0.0;
    cannonball->q()  = 0.0;
  }
  
  return;
}

bool BodyRosTankControllerItem::control()
{
  controlTime_ = controllerTarget->currentTime();

  /* control cannon */
  qref[0] = -cannon_yaw;
  qref[1] = cannon_pitch;

  for (size_t i = 0; i < body()->numJoints(); i++) {
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

  if(!cannon_fired){
    ballistic_z->q()=0;
    ballistic_y->q()=0;
    cannonball->q() =0;
  }
  
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
