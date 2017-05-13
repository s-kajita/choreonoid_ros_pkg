/**
  @file
  @author Shuuji Kajita
 */

#ifndef CNOID_ROS_PLUGIN_BODY_ROS_TANK_CONTROLLER_ITEM_H_INCLUDED
#define CNOID_ROS_PLUGIN_BODY_ROS_TANK_CONTROLLER_ITEM_H_INCLUDED

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
#include <geometry_msgs/Twist.h>

#include <vector>

#define DEBUG_ROS_TANK_CONTROLLER 0

#include <cnoid/YAMLReader>
#include <cnoid/FileUtil>

namespace cnoid {

class CNOID_EXPORT BodyRosTankControllerItem : public ControllerItem
{
public:
    static void initialize(ExtensionManager* ext);
    
    BodyRosTankControllerItem();
    BodyRosTankControllerItem(const BodyRosTankControllerItem& org);
    virtual ~BodyRosTankControllerItem();

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
      @brief Receive twist message.
      @param[in] msg twist message.
     */
    void receive_message(const geometry_msgs::Twist &twist);
    void cannon_msg(const geometry_msgs::Twist &twist);

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
    void doPutProperties(PutPropertyFunction& putProperty);

    double timeStep_;

    const Target* controllerTarget;
    double controlTime_;
    std::ostream& os;

    std::string bodyName;

    Link* crawlerL;
    Link* crawlerR;

    Vector3 lin_vel;
    Vector3 ang_vel;

    double cannon_ori;
    double cannon_ud;

    boost::shared_ptr<ros::NodeHandle> rosnode_;
    boost::shared_ptr<ros::AsyncSpinner> async_ros_spin_;

    ros::Subscriber cmd_subscriber_;
    ros::Subscriber cmd_cannon;

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
    std::string pdc_parameter_filename_;
    std::vector<double> pgain;
    std::vector<double> dgain;
    std::vector<double> u_lower;
    std::vector<double> u_upper;

    std::vector<double> qref;
    std::vector<double> q_old_;

    /**
     */
    bool set_pdc_parameters(Listing* src, std::vector<double>& dst);

    /**
     */
    bool load_pdc_parameters();

    /**
     */
    void pd_control(Link* joint, double q_ref);

};

typedef ref_ptr<BodyRosTankControllerItem> BodyRosTankControllerItemPtr;
}

#endif  /* CNOID_ROS_PLUGIN_BODY_ROS_TANK_CONTROLLER_ITEM_H_INCLUDED */
