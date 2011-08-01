/**
  *@author: Chuka Okoye
  *@email: chuka@puppetme.com
**/
#include "HomeCommManager.h"

configuration HomeCommManagerC
{
  provides
  {
    interface SplitControl;
    interface PuppetAPI;
  }
}

implementation
{
  components new CollectionSenderC(AM_REGISTER_REQUEST),
            HomeCommManagerP,
            ActiveMessageC,
            CollectionC as Collector;

  PuppetAPI = HomeCommManagerP;
  SplitControl = HomeCommManagerP;

  HomeCommManagerP.RadioControl -> ActiveMessageC;
  HomeCommManagerP.RadioSend -> CollectionSenderC;
  HomeCommManagerP.RoutingControl -> Collector;
}
