/**
  @file
  @author S.Kajita
 */

#include <cnoid/ControllerItem>
#include <cnoid/Body>
#include <cnoid/BodyItem>
#include <cnoid/Link>
#include <cnoid/Archive>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include "exportdecl.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>

#include <vector>

#define DEBUG_ROS_CANNONBALL_CONTROLLER 0

#include <cnoid/YAMLReader>
#include <cnoid/FileUtil>

namespace cnoid {

class CNOID_EXPORT BodyCannonballControllerItem : public ControllerItem
{
public:
    static void initialize(ExtensionManager* ext);
    
    BodyCannonballControllerItem();
    BodyCannonballControllerItem(const BodyCannonballControllerItem& org);
    virtual ~BodyCannonballControllerItem();

    virtual bool start(Target* target);

    virtual void input(){return;};
    virtual bool control();
    virtual void output(){return;};
    virtual void stop()
    {
      if (ros::ok()) {
	if (async_ros_spin_) {
	  async_ros_spin_->stop();
	}
	
	if (rosnode_) {
	  rosnode_->shutdown();
	}
      }
      return;
    };
    
    /**
      @brief Receive bool message.
      @param[in] msg bool message.
     */
    void receive_fire(const std_msgs::Bool &onoff);
    
    virtual double timeStep() const {
      return timeStep_;
    };

    const BodyPtr& body() const {
      return simulationBody;
    };

    double controlTime() const {
      return controlTime_;
    }

    void setModuleName(const std::string& name);

protected:
    virtual Item* doDuplicate() const;
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

    double timeStep_;

    const Target* controllerTarget;
    double controlTime_;
    std::ostream& os;

    std::string bodyName;

    boost::shared_ptr<ros::NodeHandle> rosnode_;
    boost::shared_ptr<ros::AsyncSpinner> async_ros_spin_;

    ros::Subscriber cmd_fire;

    /*
      @brief Hook of simulation start.
      This method calling in BodyRosTestControllerItem::start.
      @retval true Start controller.
      @retval false Stop controller.
     */
    virtual bool hook_of_start();

    /*
      @brief Hook of simulation start at after creation of the ROS node handle.
      This method calling in BodyRosTestControllerItem::start.
      This method can make use of the ROS node handler of 'rosnode_'.
      @retval true Start controller.
      @retval false Stop controller.
     */
    virtual bool hook_of_start_at_after_creation_rosnode();

private:
    BodyPtr simulationBody;

};

typedef ref_ptr<BodyCannonballControllerItem> BodyCannonballControllerItemPtr;
}

//------------------------------------
#include <cnoid/Plugin>

using namespace cnoid;

void BodyCannonballControllerItem::initialize(ExtensionManager* ext) { 
  static bool initialized = false;
  int argc = 0;
  char** argv;

  if (! ros::isInitialized()) {
    ros::init(argc, argv, "choreonoid");
  }

  if (! initialized) {
    ext->itemManager().registerClass<BodyCannonballControllerItem>("BodyCannonballControllerItem");
    ext->itemManager().addCreationPanel<BodyCannonballControllerItem>();
    initialized = true;
  }
}

BodyCannonballControllerItem::BodyCannonballControllerItem()
  : os(MessageView::instance()->cout())
{
  controllerTarget        = 0;
}

BodyCannonballControllerItem::BodyCannonballControllerItem(const BodyCannonballControllerItem& org)
  : ControllerItem(org),
    os(MessageView::instance()->cout())
{
  controllerTarget        = 0;
}

BodyCannonballControllerItem::~BodyCannonballControllerItem()
{
  stop();
}

//--------------------------------------------------------------------------
Item* BodyCannonballControllerItem::doDuplicate() const
{
  return new BodyCannonballControllerItem(*this);
}

bool BodyCannonballControllerItem::store(Archive& archive)
{
  return true;
}

bool BodyCannonballControllerItem::restore(const Archive& archive)
{
  return true;
}

//--------------------------------------------------------------------------
bool BodyCannonballControllerItem::start(Target* target)
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

  std::string name = body()->name();
  std::replace(name.begin(), name.end(), '-', '_');

  if (hook_of_start() == false) {
    return false;
  }

  rosnode_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(name));
  cmd_fire = rosnode_->subscribe("fire", 1, 
					&BodyCannonballControllerItem::receive_fire, this);

  if (hook_of_start_at_after_creation_rosnode() == false) {
    return false;
  }

  async_ros_spin_.reset(new ros::AsyncSpinner(0));
  async_ros_spin_->start();

  return true;
}

bool BodyCannonballControllerItem::hook_of_start()
{
#if (DEBUG_ROS_CANNONBALL_CONTROLLER > 0)
  ROS_DEBUG("%s: Called.", __PRETTY_FUNCTION__);
#endif  /* DEBUG_ROS_CANNONBALL_CONTROLLER */

  return true;
}

bool BodyCannonballControllerItem::hook_of_start_at_after_creation_rosnode()
{
#if (DEBUG_ROS_CANNONBALL_CONTROLLER > 0)
  ROS_DEBUG("%s: Called", __PRETTY_FUNCTION__);
#endif  /* DEBUG_ROS_CANNONBALL_CONTROLLER */

  return true;
}


void BodyCannonballControllerItem::receive_fire(const std_msgs::Bool &onoff)
{
  std::ostringstream os;
  os << "fire = " << onoff.data;
  MessageView::instance()->putln(os.str());

  cnoid::Vector3 trans;

  trans << 0, 0, 1.0;
  
  body()->rootLink()->setTranslation(trans);
  
  return;
}

bool BodyCannonballControllerItem::control()
{
  controlTime_ = controllerTarget->currentTime();

  return true;
}


class RosCannonballPlugin : public Plugin
{
public:
  RosCannonballPlugin() : Plugin("RosCannonball") { }
  
  virtual bool initialize() {
    BodyCannonballControllerItem::initialize(this);
    return true;
  }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(RosCannonballPlugin);
