/**
  @file
  @author
 */

#include "BodyRosTankControllerItem.h"
#include <cnoid/Plugin>

using namespace cnoid;

class RosTankPlugin : public Plugin
{
public:
  RosTankPlugin() : Plugin("RosTank") { }
  
  virtual bool initialize() {
    BodyRosTankControllerItem::initialize(this);
    return true;
  }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(RosTankPlugin);
