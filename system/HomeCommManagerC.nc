/**
  *@author: Chuka Okoye
  *@email: chuka@puppetme.com
**/
#include "PuppetMessages.h"
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
  components HomeCommManagerP;
  components ActiveMessageC;
  components CollectionC as Collector;
  components new CollectionSender(HOME_AM_ID);
  components LedsC;

  PuppetAPI = HomeCommManagerP;
  SplitControl = HomeCommManagerP;

  HomeCommManagerP.RadioControl -> ActiveMessageC;
  HomeCommManagerP.RadioSend -> CollectionSenderC;
  HomeCommManagerP.RoutingControl -> Collector;
  HomeCommManagerP.Leds -> LedsC
}
