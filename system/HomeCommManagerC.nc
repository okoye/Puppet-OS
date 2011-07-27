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
  //TODO: put some wirings here
  components HomeCommManagerP;
  components DymoNetworkC;
  
  PuppetAPI = HomeCommManagerP;
  SplitControl = HomeCommManagerP;

  HomeCommManagerP.RadioControl -> DymoNetworkC;
  HomeCommManagerP.RadioSend -> DymoNetworkC.MHSend[HOME_AM_ID];
  HomeCommManagerP.MultiHopPacket -> DymoNetworkC;
  HomeCommManagerP.Packet -> DymoNetworkC;
}
