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
  components new CollectionSenderC(AM_HOMECOMM),
            HomeCommManagerP,
            ActiveMessageC,
            CollectionC as Collector,
            LedsC;

  PuppetAPI = HomeCommManagerP;
  SplitControl = HomeCommManagerP;

  HomeCommManagerP.RadioControl -> ActiveMessageC;
  HomeCommManagerP.RadioSend -> CollectionSenderC;
  HomeCommManagerP.RoutingControl -> Collector;
  HomeCommManagerP.Leds -> LedsC;
}
