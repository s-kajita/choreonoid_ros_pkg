/**
  @file
  @author
 */

#include "BodyRosTankControllerItem.h"
#include <cnoid/Plugin>

using namespace cnoid;

class RosMinePlugin : public Plugin
{
public:
  RosMinePlugin() : Plugin("RosTank") { }
  
  virtual bool initialize() {
    BodyRosTankControllerItem::initialize(this);
    return true;
  }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(RosMinePlugin);
